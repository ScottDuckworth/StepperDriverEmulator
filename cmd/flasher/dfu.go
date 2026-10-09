package main

import (
	"bytes"
	"fmt"
	"time"

	usb "github.com/kevmo314/go-usb"
)

const (
	STM32_VID     = 0x0483
	STM32_CDC_PID = 0x5740
	STM32_DFU_PID = 0xDF11

	DFU_DETACH    = 0
	DFU_DNLOAD    = 1
	DFU_UPLOAD    = 2
	DFU_GETSTATUS = 3
	DFU_CLRSTATUS = 4
	DFU_GETSTATE  = 5
	DFU_ABORT     = 6

	DFU_STATE_APP_IDLE            = 0
	DFU_STATE_APP_DETACH          = 1
	DFU_STATE_DFU_IDLE            = 2
	DFU_STATE_DFU_DNLOAD_SYNC     = 3
	DFU_STATE_DFU_DNBUSY          = 4
	DFU_STATE_DFU_DNLOAD_IDLE     = 5
	DFU_STATE_DFU_MANIFEST_SYNC   = 6
	DFU_STATE_DFU_MANIFEST        = 7
	DFU_STATE_DFU_MANIFEST_WAIT_RESET = 8
	DFU_STATE_DFU_UPLOAD_IDLE     = 9
	DFU_STATE_DFU_ERROR           = 10

	DFUSE_CMD_SET_ADDRESS = 0x21
	DFUSE_CMD_ERASE_PAGE  = 0x41

	FLASH_BASE_ADDR     = uint32(0x08000000)
	FLASH_PAGE_SIZE     = 1024
	FLASH_APP_MAX_PAGES = 31 // Pages 0-30 (31 KB)
	FLASH_APP_MAX_BYTES = FLASH_APP_MAX_PAGES * FLASH_PAGE_SIZE
	FLASH_TOTAL_BYTES   = 32 * FLASH_PAGE_SIZE
)

type DfuStatus struct {
	Status      byte
	PollTimeout time.Duration
	State       byte
	StringIndex byte
}

type DfuClient struct {
	dev    *usb.Device
	handle *DfuPlatformHandle
}

func OpenDfuDevice(dev *usb.Device) (*DfuClient, error) {
	h, err := openPlatformDfuHandle(dev)
	if err != nil {
		return nil, fmt.Errorf("could not open DFU USB device: %w", err)
	}

	return &DfuClient{
		dev:    dev,
		handle: h,
	}, nil
}

func (c *DfuClient) Close() {
	if c.handle != nil {
		c.handle.Close()
		c.handle = nil
	}
}

func (c *DfuClient) GetStatus() (*DfuStatus, error) {
	buf := make([]byte, 6)
	_, err := c.handle.ControlTransfer(0xA1, DFU_GETSTATUS, 0, 0, buf, 5*time.Second)
	if err != nil {
		return nil, err
	}

	pollMs := uint32(buf[1]) | (uint32(buf[2]) << 8) | (uint32(buf[3]) << 16)
	return &DfuStatus{
		Status:      buf[0],
		PollTimeout: time.Duration(pollMs) * time.Millisecond,
		State:       buf[4],
		StringIndex: buf[5],
	}, nil
}

func (c *DfuClient) ClearStatus() error {
	_, err := c.handle.ControlTransfer(0x21, DFU_CLRSTATUS, 0, 0, nil, 5*time.Second)
	return err
}

func (c *DfuClient) Abort() error {
	_, err := c.handle.ControlTransfer(0x21, DFU_ABORT, 0, 0, nil, 5*time.Second)
	return err
}

func (c *DfuClient) WaitState(expectedState byte, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		status, err := c.GetStatus()
		if err != nil {
			return fmt.Errorf("GETSTATUS failed: %w", err)
		}

		if status.Status != 0 {
			_ = c.ClearStatus()
			return fmt.Errorf("DFU error status %d in state %d", status.Status, status.State)
		}

		if status.State == expectedState {
			return nil
		}

		sleepTime := status.PollTimeout
		if sleepTime < 5*time.Millisecond {
			sleepTime = 5 * time.Millisecond
		}
		time.Sleep(sleepTime)
	}
	return fmt.Errorf("timed out waiting for DFU state %d", expectedState)
}

func (c *DfuClient) SetAddress(addr uint32) error {
	cmd := []byte{
		DFUSE_CMD_SET_ADDRESS,
		byte(addr),
		byte(addr >> 8),
		byte(addr >> 16),
		byte(addr >> 24),
	}
	_, err := c.handle.ControlTransfer(0x21, DFU_DNLOAD, 0, 0, cmd, 5*time.Second)
	if err != nil {
		return fmt.Errorf("SET_ADDRESS DNLOAD failed: %w", err)
	}
	return c.WaitState(DFU_STATE_DFU_DNLOAD_IDLE, 5*time.Second)
}

func (c *DfuClient) ErasePage(addr uint32) error {
	cmd := []byte{
		DFUSE_CMD_ERASE_PAGE,
		byte(addr),
		byte(addr >> 8),
		byte(addr >> 16),
		byte(addr >> 24),
	}
	_, err := c.handle.ControlTransfer(0x21, DFU_DNLOAD, 0, 0, cmd, 5*time.Second)
	if err != nil {
		return fmt.Errorf("ERASE_PAGE DNLOAD failed: %w", err)
	}
	return c.WaitState(DFU_STATE_DFU_DNLOAD_IDLE, 10*time.Second)
}

func (c *DfuClient) WriteBlock(blockNum uint16, data []byte) error {
	_, err := c.handle.ControlTransfer(0x21, DFU_DNLOAD, blockNum, 0, data, 5*time.Second)
	if err != nil {
		return fmt.Errorf("write block %d failed: %w", blockNum, err)
	}
	return c.WaitState(DFU_STATE_DFU_DNLOAD_IDLE, 5*time.Second)
}

func (c *DfuClient) ReadBlock(blockNum uint16, length int) ([]byte, error) {
	buf := make([]byte, length)
	n, err := c.handle.ControlTransfer(0xA1, DFU_UPLOAD, blockNum, 0, buf, 5*time.Second)
	if err != nil {
		return nil, fmt.Errorf("read block %d failed: %w", blockNum, err)
	}
	return buf[:n], nil
}

func (c *DfuClient) Leave(jumpAddr uint32) error {
	if err := c.SetAddress(jumpAddr); err != nil {
		return err
	}
	// Download 0-length block to trigger manifest phase
	_, _ = c.handle.ControlTransfer(0x21, DFU_DNLOAD, 0, 0, nil, 5*time.Second)
	_, _ = c.GetStatus()
	return nil
}

func FindDfuDevice() (*usb.Device, error) {
	devices, err := usb.DeviceList()
	if err != nil {
		return nil, fmt.Errorf("usb.DeviceList failed: %w", err)
	}

	for _, dev := range devices {
		if dev.Descriptor.VendorID == STM32_VID && dev.Descriptor.ProductID == STM32_DFU_PID {
			return dev, nil
		}
	}
	return nil, nil
}

func WaitForDfuDevice(timeout time.Duration, verbose bool) (*usb.Device, error) {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		dev, err := FindDfuDevice()
		if err == nil && dev != nil {
			return dev, nil
		}
		time.Sleep(250 * time.Millisecond)
	}
	return nil, fmt.Errorf("timed out waiting for STM32 DFU device (0483:DF11) to enumerate")
}

func FlashFirmware(client *DfuClient, binData []byte, fullErase bool, reboot bool, verbose bool) error {
	numPages := (len(binData) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE
	if fullErase {
		numPages = 32
	}

	if verbose {
		fmt.Printf("Initial DFU state check...\n")
	}
	st, err := client.GetStatus()
	if err != nil {
		return fmt.Errorf("failed to get initial DFU status: %w", err)
	}
	if verbose {
		fmt.Printf("Initial DFU State: %d, Status: %d\n", st.State, st.Status)
	}

	// Ensure device is in dfuIDLE before beginning
	for st.State != DFU_STATE_DFU_IDLE {
		if st.State == DFU_STATE_DFU_ERROR {
			if verbose {
				fmt.Println("Clearing previous DFU error state...")
			}
			_ = client.ClearStatus()
		} else {
			if verbose {
				fmt.Printf("Aborting previous state %d to return to dfuIDLE...\n", st.State)
			}
			_ = client.Abort()
		}
		_ = client.WaitState(DFU_STATE_DFU_IDLE, 2*time.Second)
		st, err = client.GetStatus()
		if err != nil || st.State == DFU_STATE_DFU_IDLE {
			break
		}
	}

	// 1. Erase Flash Pages
	fmt.Printf("Erasing %d Flash page(s)...\n", numPages)
	for i := 0; i < numPages; i++ {
		pageAddr := FLASH_BASE_ADDR + uint32(i*FLASH_PAGE_SIZE)
		if verbose {
			fmt.Printf("  Erasing page %2d/%2d at 0x%08X...\n", i+1, numPages, pageAddr)
		} else {
			fmt.Printf("\r  Erase progress: %d/%d pages", i+1, numPages)
		}
		if err := client.ErasePage(pageAddr); err != nil {
			fmt.Println()
			return fmt.Errorf("failed to erase page %d (0x%08X): %w", i, pageAddr, err)
		}
	}
	fmt.Printf("\r  Erase complete: %d pages erased.      \n", numPages)

	// 2. Program Firmware
	const chunkSize = 2048
	totalBytes := len(binData)
	totalBlocks := (totalBytes + chunkSize - 1) / chunkSize

	fmt.Printf("Programming %d bytes across %d block(s)...\n", totalBytes, totalBlocks)

	for block := 0; block < totalBlocks; block++ {
		start := block * chunkSize
		end := start + chunkSize
		if end > totalBytes {
			end = totalBytes
		}
		chunk := binData[start:end]
		blockAddr := FLASH_BASE_ADDR + uint32(start)

		if err := client.SetAddress(blockAddr); err != nil {
			fmt.Println()
			return fmt.Errorf("failed to set address 0x%08X: %w", blockAddr, err)
		}

		if err := client.WriteBlock(2, chunk); err != nil {
			fmt.Println()
			return fmt.Errorf("programming failed at block %d (address 0x%08X): %w", block, blockAddr, err)
		}

		pct := float64(end) * 100.0 / float64(totalBytes)
		fmt.Printf("\r  Programming: %5.1f%% (%d/%d bytes)", pct, end, totalBytes)
	}
	fmt.Println("\n  Programming complete.")

	// 3. Verify Firmware
	fmt.Printf("Verifying programmed data...\n")
	if err := client.SetAddress(FLASH_BASE_ADDR); err != nil {
		return fmt.Errorf("failed to set address for verification: %w", err)
	}
	_ = client.Abort()
	if err := client.WaitState(DFU_STATE_DFU_IDLE, 2*time.Second); err != nil {
		_ = client.ClearStatus()
		_ = client.WaitState(DFU_STATE_DFU_IDLE, 2*time.Second)
	}

	for block := 0; block < totalBlocks; block++ {
		start := block * chunkSize
		end := start + chunkSize
		if end > totalBytes {
			end = totalBytes
		}
		chunk := binData[start:end]
		blockNum := uint16(block + 2)

		readBack, err := client.ReadBlock(blockNum, len(chunk))
		if err != nil {
			fmt.Println()
			return fmt.Errorf("verification read failed at block %d: %w", block, err)
		}

		if !bytes.Equal(chunk, readBack) {
			fmt.Println()
			fmt.Printf("Expected first 16 bytes: % X\n", chunk[:min(16, len(chunk))])
			fmt.Printf("Actual read 16 bytes:    % X\n", readBack[:min(16, len(readBack))])
			return fmt.Errorf("verification mismatch at block %d", block)
		}

		pct := float64(end) * 100.0 / float64(totalBytes)
		fmt.Printf("\r  Verifying:   %5.1f%% (%d/%d bytes)", pct, end, totalBytes)
	}
	fmt.Println("\n  Verification successful: All bytes match perfectly!")

	// Exit upload state back to dfuIDLE
	_ = client.Abort()
	_ = client.WaitState(DFU_STATE_DFU_IDLE, 2*time.Second)

	// 4. Reboot into application
	if reboot {
		fmt.Printf("Rebooting device into application...\n")
		_ = client.Leave(FLASH_BASE_ADDR)
		fmt.Printf("Device reboot triggered.\n")
	}

	return nil
}
