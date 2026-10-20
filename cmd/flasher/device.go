package main

import (
	"bufio"
	"fmt"
	"strings"
	"time"

	usb "github.com/kevmo314/go-usb"
	"go.bug.st/serial"
	"go.bug.st/serial/enumerator"
)

type DeviceInfo struct {
	Port            string
	VID             string
	PID             string
	SerialNumber    string
	Product         string
	LogicalName     string
	FirmwareVersion string
	IsDfu           bool
	IsEmulator      bool
	IsStLink        bool
}

// FindStepperUsbDevices scans USB devices directly via go-usb to find any devices
// reporting a product string of "StepperDriverEmulator" (USBD_PRODUCT_STRING_FS).
func FindStepperUsbDevices(verbose bool) ([]*DeviceInfo, error) {
	devices, err := usb.DeviceList()
	if err != nil {
		return nil, fmt.Errorf("usb.DeviceList failed: %w", err)
	}

	var found []*DeviceInfo
	for _, dev := range devices {
		desc := dev.Descriptor
		if desc.VendorID != STM32_VID {
			continue
		}

		handle, err := dev.Open()
		if err != nil {
			continue
		}

		var product, serialNumber string
		if desc.ProductIndex > 0 {
			product, _ = handle.StringDescriptor(desc.ProductIndex)
		}
		if desc.SerialNumberIndex > 0 {
			serialNumber, _ = handle.StringDescriptor(desc.SerialNumberIndex)
		}
		handle.Close()

		if verbose {
			fmt.Printf("  USB device 0483:%04x: Product='%s', S/N='%s'\n", desc.ProductID, product, serialNumber)
		}

		if strings.Contains(strings.ToLower(product), "stepperdriveremulator") {
			found = append(found, &DeviceInfo{
				VID:          fmt.Sprintf("%04x", desc.VendorID),
				PID:          fmt.Sprintf("%04x", desc.ProductID),
				SerialNumber: serialNumber,
				Product:      product,
				IsEmulator:   true,
			})
		}
	}
	return found, nil
}

// ProbeSerialDevice opens a serial port and issues 'name' and 'version' commands.
// It returns the logical name and firmware version if reported by the device.
func ProbeSerialDevice(portName string, timeout time.Duration) (string, string, error) {
	mode := &serial.Mode{
		BaudRate: 115200,
	}
	port, err := serial.Open(portName, mode)
	if err != nil {
		return "", "", err
	}
	defer port.Close()

	if err := port.SetReadTimeout(timeout); err != nil {
		return "", "", err
	}

	_, err = port.Write([]byte("\r\nname\r\nversion\r\n"))
	if err != nil {
		return "", "", err
	}

	reader := bufio.NewReader(port)
	deadline := time.Now().Add(timeout)
	name := ""
	version := ""

	for time.Now().Before(deadline) {
		line, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		line = strings.TrimSpace(line)
		if strings.HasPrefix(line, "name ") {
			name = strings.TrimSpace(strings.TrimPrefix(line, "name "))
		} else if strings.HasPrefix(line, "version ") {
			version = strings.TrimSpace(strings.TrimPrefix(line, "version "))
		}
		if name != "" && version != "" {
			break
		}
	}

	if name == "" && version == "" {
		return "", "", fmt.Errorf("no response received")
	}

	return name, version, nil
}

// FindStepperSerialPorts scans available serial ports cross-platform using go.bug.st/serial.
// It prioritizes ports whose Product string or active handshake confirms StepperDriverEmulator.
func FindStepperSerialPorts(probeHandshake bool, verbose bool) ([]*DeviceInfo, error) {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, fmt.Errorf("failed to enumerate serial ports: %w", err)
	}

	var emulators []*DeviceInfo

	for _, p := range ports {
		if !p.IsUSB {
			continue
		}

		vid := strings.ToLower(p.VID)
		pid := strings.ToLower(p.PID)
		prod := p.Product
		sn := p.SerialNumber

		isStLink := (vid == "0483" && strings.HasPrefix(pid, "374")) ||
			(vid == "0483" && strings.HasPrefix(pid, "375")) ||
			strings.Contains(strings.ToLower(prod), "stlink") ||
			strings.Contains(strings.ToLower(prod), "st-link")

		if isStLink {
			if verbose {
				fmt.Printf("  Found ST-Link probe on %s (ignored)\n", p.Name)
			}
			continue
		}

		isMatch := strings.Contains(strings.ToLower(prod), "stepperdriveremulator") ||
			(vid == "0483" && pid == "5740")

		info := &DeviceInfo{
			Port:         p.Name,
			VID:          vid,
			PID:          pid,
			SerialNumber: sn,
			Product:      prod,
			IsEmulator:   isMatch,
		}

		if isMatch && probeHandshake {
			name, ver, err := ProbeSerialDevice(p.Name, 250*time.Millisecond)
			if err == nil {
				info.LogicalName = name
				info.FirmwareVersion = ver
				info.IsEmulator = true
				if verbose {
					fmt.Printf("  Verified StepperDriverEmulator on %s (Name: '%s', Version: '%s')\n", p.Name, name, ver)
				}
			}
		}

		if info.IsEmulator {
			emulators = append(emulators, info)
		}
	}

	return emulators, nil
}

// SendDfuCommand connects to the emulator's serial port and issues the 'dfu' command.
func SendDfuCommand(portName string, verbose bool) error {
	if verbose {
		fmt.Printf("Connecting to %s to send 'dfu' command...\n", portName)
	}

	mode := &serial.Mode{
		BaudRate: 115200,
	}
	var port serial.Port
	var err error
	for attempt := 0; attempt < 10; attempt++ {
		port, err = serial.Open(portName, mode)
		if err == nil {
			break
		}
		time.Sleep(100 * time.Millisecond)
	}
	if err != nil {
		return fmt.Errorf("could not open serial port %s: %w", portName, err)
	}
	defer port.Close()

	_ = port.SetReadTimeout(500 * time.Millisecond)

	// Send \r\n first to clear any stale buffer in cmd.c, then send dfu\r\n
	_, err = port.Write([]byte("\r\ndfu\r\n"))
	if err != nil {
		return fmt.Errorf("failed to write 'dfu' command to %s: %w", portName, err)
	}

	// Drain responses looking for "dfu ok"
	reader := bufio.NewReader(port)
	deadline := time.Now().Add(1 * time.Second)
	for time.Now().Before(deadline) {
		line, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		trimmed := strings.TrimSpace(line)
		if verbose && trimmed != "" {
			fmt.Printf("Response from %s: %s\n", portName, trimmed)
		}
		if strings.Contains(trimmed, "dfu ok") {
			break
		}
	}

	return nil
}

