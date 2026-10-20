package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"time"

	usb "github.com/kevmo314/go-usb"
)

func resolveBinaryPath(specifiedPath string) (string, error) {
	if specifiedPath != "" {
		if _, err := os.Stat(specifiedPath); err == nil {
			return specifiedPath, nil
		}
		return "", fmt.Errorf("specified binary not found: %s", specifiedPath)
	}

	candidates := []string{
		"build/Release/StepperDriverEmulator.bin",
		"StepperDriverEmulator.bin",
		"../build/Release/StepperDriverEmulator.bin",
		"../../build/Release/StepperDriverEmulator.bin",
	}

	for _, c := range candidates {
		if _, err := os.Stat(c); err == nil {
			return c, nil
		}
	}

	return "", fmt.Errorf("firmware binary not found in standard build locations. Please specify with -bin <path>")
}

func formatDeviceInfo(emu *DeviceInfo) (string, string, string) {
	nameStr := ""
	if emu.LogicalName != "" {
		nameStr = fmt.Sprintf(" (Name: '%s')", emu.LogicalName)
	}
	verStr := ""
	if emu.FirmwareVersion != "" {
		verStr = fmt.Sprintf(" [%s]", emu.FirmwareVersion)
	}
	snStr := ""
	if emu.SerialNumber != "" {
		snStr = fmt.Sprintf(" [S/N: %s]", emu.SerialNumber)
	}
	return nameStr, verStr, snStr
}

func flashDfuDevice(dev *usb.Device, binData []byte, fullErase bool, reboot bool, verbose bool) error {
	client, err := OpenDfuDevice(dev)
	if err != nil {
		return fmt.Errorf("failed to open DFU device: %w", err)
	}
	defer client.Close()

	if err := FlashFirmware(client, binData, fullErase, reboot, verbose); err != nil {
		return err
	}

	if reboot {
		_ = WaitForDfuDeviceDisconnect(3*time.Second, verbose)
	}
	return nil
}

func flashCdcPort(portName string, binData []byte, fullErase bool, reboot bool, verbose bool) error {
	fmt.Printf("Sending 'dfu' command to %s...\n", portName)
	if err := SendDfuCommand(portName, verbose); err != nil {
		return fmt.Errorf("failed to send 'dfu' command to %s: %w", portName, err)
	}

	fmt.Println("Waiting for STM32 DFU Bootloader enumeration...")
	dfuDev, err := WaitForDfuDevice(8*time.Second, verbose)
	if err != nil {
		return fmt.Errorf("%w\nCheck hardware connection", err)
	}
	fmt.Println("STM32 DFU Bootloader connected.")

	return flashDfuDevice(dfuDev, binData, fullErase, reboot, verbose)
}

func main() {
	var portFlag string
	var binFlag string
	var fullEraseFlag bool
	var noRebootFlag bool
	var verboseFlag bool

	flag.StringVar(&portFlag, "port", "", "Serial Virtual COM Port (e.g. COM3 or /dev/ttyACM0). Auto-detected if omitted.")
	flag.StringVar(&portFlag, "p", "", "Shorthand for -port")
	flag.StringVar(&binFlag, "bin", "", "Path to firmware .bin file (default: build/Release/StepperDriverEmulator.bin)")
	flag.StringVar(&binFlag, "b", "", "Shorthand for -bin")
	flag.BoolVar(&fullEraseFlag, "full-erase", false, "Erase all 32 KB flash (default preserves Page 31 configuration)")
	flag.BoolVar(&noRebootFlag, "no-reboot", false, "Leave device in DFU mode after flashing instead of auto-rebooting")
	flag.BoolVar(&verboseFlag, "v", false, "Enable verbose output")
	flag.BoolVar(&verboseFlag, "verbose", false, "Enable verbose output")

	flag.Usage = func() {
		fmt.Fprintf(os.Stderr, "USB DFU Firmware Flasher for StepperDriverEmulator (STM32F042)\n\nUsage:\n")
		flag.PrintDefaults()
	}

	flag.Parse()

	// 1. Locate and validate firmware image
	binPath, err := resolveBinaryPath(binFlag)
	if err != nil {
		fmt.Fprintf(os.Stderr, "[ERROR] %v\n", err)
		os.Exit(1)
	}

	binData, err := os.ReadFile(binPath)
	if err != nil {
		fmt.Fprintf(os.Stderr, "[ERROR] Failed to read binary: %v\n", err)
		os.Exit(1)
	}

	absPath, _ := filepath.Abs(binPath)
	fmt.Printf("Firmware image: %s (%d bytes)\n", filepath.Base(absPath), len(binData))

	if len(binData) > int(FLASH_TOTAL_BYTES) {
		fmt.Fprintf(os.Stderr, "[ERROR] Image size (%d B) exceeds physical flash capacity (%d B)!\n",
			len(binData), FLASH_TOTAL_BYTES)
		os.Exit(1)
	}

	if !fullEraseFlag && len(binData) > int(FLASH_APP_MAX_BYTES) {
		fmt.Fprintf(os.Stderr, "[ERROR] Binary size (%d B) exceeds application partition (%d B).\n"+
			"Flashing this image would overwrite persistent configuration on Page 31.\n"+
			"Pass -full-erase if you explicitly intend to overwrite configuration storage.\n",
			len(binData), FLASH_APP_MAX_BYTES)
		os.Exit(1)
	}

	reboot := !noRebootFlag

	// 2. Determine target device(s)
	if portFlag != "" {
		fmt.Printf("Targeting specified serial port: %s\n", portFlag)
		if err := flashCdcPort(portFlag, binData, fullEraseFlag, reboot, verboseFlag); err != nil {
			fmt.Fprintf(os.Stderr, "\n[ERROR] Flashing failed: %v\n", err)
			os.Exit(1)
		}
		fmt.Println("\nFirmware update completed successfully!")
		return
	}

	// Auto-detect mode: scan for DFU devices and CDC serial ports
	dfuDevs, err := FindDfuDevices()
	if err != nil && verboseFlag {
		fmt.Printf("Note during DFU search: %v\n", err)
	}

	if verboseFlag {
		fmt.Println("Scanning for attached StepperDriverEmulator devices...")
	}

	emulators, err := FindStepperSerialPorts(true, verboseFlag)
	if err != nil && verboseFlag {
		fmt.Printf("Note during port search: %v\n", err)
	}

	if len(dfuDevs) == 0 && len(emulators) == 0 {
		usbDevs, _ := FindStepperUsbDevices(verboseFlag)
		if len(usbDevs) > 0 {
			fmt.Printf("Found StepperDriverEmulator on USB (%s), but no active COM port responded.\n", usbDevs[0].Product)
		}

		fmt.Println("No active emulator CDC port found. Checking if board is already in DFU mode...")
		dfuDev, _ := WaitForDfuDevice(2*time.Second, verboseFlag)
		if dfuDev == nil {
			fmt.Fprintf(os.Stderr, "[ERROR] No StepperDriverEmulator device found and no STM32 DFU device detected.\n"+
				"Please plug in the device.\n")
			os.Exit(1)
		}
		dfuDevs = append(dfuDevs, dfuDev)
	}

	totalTargets := len(dfuDevs) + len(emulators)

	if totalTargets == 1 {
		if len(dfuDevs) == 1 {
			fmt.Println("Found 1 StepperDriverEmulator device in DFU mode (0483:DF11).")
			if err := flashDfuDevice(dfuDevs[0], binData, fullEraseFlag, reboot, verboseFlag); err != nil {
				fmt.Fprintf(os.Stderr, "\n[ERROR] Flashing failed: %v\n", err)
				os.Exit(1)
			}
		} else {
			emu := emulators[0]
			nameStr, verStr, snStr := formatDeviceInfo(emu)
			fmt.Printf("Auto-selected StepperDriverEmulator: %s%s%s%s\n", emu.Port, nameStr, verStr, snStr)
			if err := flashCdcPort(emu.Port, binData, fullEraseFlag, reboot, verboseFlag); err != nil {
				fmt.Fprintf(os.Stderr, "\n[ERROR] Flashing failed: %v\n", err)
				os.Exit(1)
			}
		}
		fmt.Println("\nFirmware update completed successfully!")
		return
	}

	// Multiple devices detected: flash all sequentially
	fmt.Printf("Found %d StepperDriverEmulator devices:\n", totalTargets)
	for range dfuDevs {
		fmt.Println("  - [DFU Bootloader] (0483:DF11)")
	}
	for _, emu := range emulators {
		nameStr, verStr, snStr := formatDeviceInfo(emu)
		fmt.Printf("  - %s%s%s%s\n", emu.Port, nameStr, verStr, snStr)
	}

	targetIndex := 1
	for _, dev := range dfuDevs {
		fmt.Printf("\n[%d/%d] Flashing STM32 DFU Bootloader device...\n", targetIndex, totalTargets)
		if err := flashDfuDevice(dev, binData, fullEraseFlag, reboot, verboseFlag); err != nil {
			fmt.Fprintf(os.Stderr, "\n[ERROR] Flashing failed on device %d: %v\n", targetIndex, err)
			os.Exit(1)
		}
		targetIndex++
		time.Sleep(500 * time.Millisecond)
	}

	for _, emu := range emulators {
		nameStr, verStr, snStr := formatDeviceInfo(emu)
		fmt.Printf("\n[%d/%d] Flashing %s%s%s%s...\n", targetIndex, totalTargets, emu.Port, nameStr, verStr, snStr)
		if err := flashCdcPort(emu.Port, binData, fullEraseFlag, reboot, verboseFlag); err != nil {
			fmt.Fprintf(os.Stderr, "\n[ERROR] Flashing failed on device %d (%s): %v\n", targetIndex, emu.Port, err)
			os.Exit(1)
		}
		targetIndex++
		time.Sleep(500 * time.Millisecond)
	}

	fmt.Printf("\nAll %d devices updated successfully!\n", totalTargets)
}

