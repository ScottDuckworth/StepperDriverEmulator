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
	Port         string
	VID          string
	PID          string
	SerialNumber string
	Product      string
	LogicalName  string
	IsDfu        bool
	IsEmulator   bool
	IsStLink     bool
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

// ProbeSerialDeviceName opens a serial port and issues the 'name' command.
// If the device responds with 'name <NAME>', it returns the verified name.
func ProbeSerialDeviceName(portName string, timeout time.Duration) (string, error) {
	mode := &serial.Mode{
		BaudRate: 115200,
	}
	port, err := serial.Open(portName, mode)
	if err != nil {
		return "", err
	}
	defer port.Close()

	if err := port.SetReadTimeout(timeout); err != nil {
		return "", err
	}

	_, err = port.Write([]byte("name\r\n"))
	if err != nil {
		return "", err
	}

	reader := bufio.NewReader(port)
	deadline := time.Now().Add(timeout)

	for time.Now().Before(deadline) {
		line, err := reader.ReadString('\n')
		if err != nil {
			break
		}
		line = strings.TrimSpace(line)
		if strings.HasPrefix(line, "name ") {
			return strings.TrimSpace(strings.TrimPrefix(line, "name ")), nil
		}
	}

	return "", fmt.Errorf("no 'name' response received")
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
			logicalName, err := ProbeSerialDeviceName(p.Name, 150*time.Millisecond)
			if err == nil {
				info.LogicalName = logicalName
				info.IsEmulator = true
				if verbose {
					fmt.Printf("  Verified StepperDriverEmulator on %s (Name: '%s')\n", p.Name, logicalName)
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
	port, err := serial.Open(portName, mode)
	if err != nil {
		return fmt.Errorf("could not open serial port %s: %w", portName, err)
	}
	defer port.Close()

	_ = port.SetReadTimeout(500 * time.Millisecond)

	_, err = port.Write([]byte("dfu\r\n"))
	if err != nil {
		return fmt.Errorf("failed to write 'dfu' command to %s: %w", portName, err)
	}

	// Read ack if returned
	buf := make([]byte, 128)
	n, _ := port.Read(buf)
	if verbose && n > 0 {
		fmt.Printf("Response from device: %s\n", strings.TrimSpace(string(buf[:n])))
	}

	return nil
}

