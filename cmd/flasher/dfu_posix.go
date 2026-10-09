//go:build !windows

package main

import (
	"fmt"
	"time"

	usb "github.com/kevmo314/go-usb"
)

type DfuPlatformHandle struct {
	handle *usb.DeviceHandle
}

func openPlatformDfuHandle(dev *usb.Device) (*DfuPlatformHandle, error) {
	handle, err := dev.Open()
	if err != nil {
		return nil, fmt.Errorf("could not open DFU device: %w", err)
	}

	return &DfuPlatformHandle{
		handle: handle,
	}, nil
}

func (h *DfuPlatformHandle) Close() {
	if h.handle != nil {
		h.handle.Close()
		h.handle = nil
	}
}

func (h *DfuPlatformHandle) ControlTransfer(requestType, request uint8, value, index uint16, data []byte, timeout time.Duration) (int, error) {
	return h.handle.ControlTransfer(requestType, request, value, index, data, timeout)
}

