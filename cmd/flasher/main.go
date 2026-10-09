package main

import (
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"time"
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

	// 2. Check if device is already in DFU mode
	dfuDev, err := FindDfuDevice()
	if err != nil && verboseFlag {
		fmt.Printf("Note during DFU check: %v\n", err)
	}

	if dfuDev != nil {
		fmt.Println("STM32 DFU Bootloader (0483:DF11) detected.")
	} else {
		// 3. Select serial port and issue 'dfu' reset command
		targetPort := portFlag

		if targetPort == "" {
			if verboseFlag {
				fmt.Println("Scanning for attached StepperDriverEmulator devices...")
			}

			emulators, err := FindStepperSerialPorts(true, verboseFlag)
			if err != nil && verboseFlag {
				fmt.Printf("Note during port search: %v\n", err)
			}

			if len(emulators) == 1 {
				targetPort = emulators[0].Port
				nameStr := ""
				if emulators[0].LogicalName != "" {
					nameStr = fmt.Sprintf(" (Name: '%s')", emulators[0].LogicalName)
				}
				snStr := ""
				if emulators[0].SerialNumber != "" {
					snStr = fmt.Sprintf(" [S/N: %s]", emulators[0].SerialNumber)
				}
				fmt.Printf("Auto-selected StepperDriverEmulator: %s%s%s\n", targetPort, nameStr, snStr)
			} else if len(emulators) > 1 {
				fmt.Printf("Found %d StepperDriverEmulator devices:\n", len(emulators))
				for _, emu := range emulators {
					nameStr := ""
					if emu.LogicalName != "" {
						nameStr = fmt.Sprintf(" Name: '%s'", emu.LogicalName)
					}
					snStr := ""
					if emu.SerialNumber != "" {
						snStr = fmt.Sprintf(" S/N: %s", emu.SerialNumber)
					}
					fmt.Printf("  - %s%s%s\n", emu.Port, nameStr, snStr)
				}
				fmt.Fprintf(os.Stderr, "\nPlease specify which device to flash using -port <PORT>.\n")
				os.Exit(1)
			} else {
				// No emulator CDC port detected, check if already in DFU mode or check raw USB
				usbDevs, _ := FindStepperUsbDevices(verboseFlag)
				if len(usbDevs) > 0 {
					fmt.Printf("Found StepperDriverEmulator on USB (%s), but no active COM port responded.\n", usbDevs[0].Product)
				}

				fmt.Println("No active emulator CDC port found. Checking if board is already in DFU mode...")
				dfuDev, _ = WaitForDfuDevice(2*time.Second, verboseFlag)
				if dfuDev == nil {
					fmt.Fprintf(os.Stderr, "[ERROR] No StepperDriverEmulator device found and no STM32 DFU device detected.\n"+
						"Please plug in the device.\n")
					os.Exit(1)
				}
			}
		}

		if dfuDev == nil {
			fmt.Printf("Sending 'dfu' command to %s...\n", targetPort)
			if err := SendDfuCommand(targetPort, verboseFlag); err != nil && verboseFlag {
				fmt.Printf("Note: %v\n", err)
			}

			fmt.Println("Waiting for STM32 DFU Bootloader enumeration...")
			dfuDev, err = WaitForDfuDevice(8*time.Second, verboseFlag)
			if err != nil {
				fmt.Fprintf(os.Stderr, "[ERROR] %v\nCheck hardware connection.\n", err)
				os.Exit(1)
			}
			fmt.Println("STM32 DFU Bootloader connected.")
		}
	}

	// 4. Connect to DFU device and program firmware
	fmt.Println("Initiating firmware programming...")
	client, err := OpenDfuDevice(dfuDev)
	if err != nil {
		fmt.Fprintf(os.Stderr, "[ERROR] Failed to open DFU device: %v\n", err)
		os.Exit(1)
	}
	defer client.Close()

	if err := FlashFirmware(client, binData, fullEraseFlag, reboot, verboseFlag); err != nil {
		fmt.Fprintf(os.Stderr, "\n[ERROR] Flashing failed: %v\n", err)
		os.Exit(1)
	}

	fmt.Println("\nFirmware update completed successfully!")
}

