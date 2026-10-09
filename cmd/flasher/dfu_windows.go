//go:build windows

package main

import (
	"fmt"
	"syscall"
	"time"
	"unsafe"

	usb "github.com/kevmo314/go-usb"
	"golang.org/x/sys/windows"
)

var (
	modwinusb                  = windows.NewLazySystemDLL("winusb.dll")
	procWinUsb_Initialize     = modwinusb.NewProc("WinUsb_Initialize")
	procWinUsb_Free           = modwinusb.NewProc("WinUsb_Free")
	procWinUsb_ControlTransfer = modwinusb.NewProc("WinUsb_ControlTransfer")
)

type winusbSetupPacket struct {
	RequestType uint8
	Request     uint8
	Value       uint16
	Index       uint16
	Length      uint16
}

type DfuPlatformHandle struct {
	fileHandle   windows.Handle
	winusbHandle uintptr
}

func openPlatformDfuHandle(dev *usb.Device) (*DfuPlatformHandle, error) {
	pathPtr, err := windows.UTF16PtrFromString(dev.Path)
	if err != nil {
		return nil, fmt.Errorf("invalid device path %s: %w", dev.Path, err)
	}

	fileHandle, err := windows.CreateFile(
		pathPtr,
		windows.GENERIC_READ|windows.GENERIC_WRITE,
		windows.FILE_SHARE_READ|windows.FILE_SHARE_WRITE,
		nil,
		windows.OPEN_EXISTING,
		windows.FILE_ATTRIBUTE_NORMAL|windows.FILE_FLAG_OVERLAPPED,
		0,
	)
	if err != nil {
		return nil, fmt.Errorf("CreateFile failed for DFU device (%s): %w", dev.Path, err)
	}

	var winusbHandle uintptr
	r0, _, e1 := procWinUsb_Initialize.Call(uintptr(fileHandle), uintptr(unsafe.Pointer(&winusbHandle)))
	if r0 == 0 {
		windows.CloseHandle(fileHandle)
		return nil, fmt.Errorf("WinUsb_Initialize failed: %w", e1)
	}

	return &DfuPlatformHandle{
		fileHandle:   fileHandle,
		winusbHandle: winusbHandle,
	}, nil
}

func (h *DfuPlatformHandle) Close() {
	if h.winusbHandle != 0 {
		procWinUsb_Free.Call(h.winusbHandle)
		h.winusbHandle = 0
	}
	if h.fileHandle != windows.InvalidHandle && h.fileHandle != 0 {
		windows.CloseHandle(h.fileHandle)
		h.fileHandle = windows.InvalidHandle
	}
}

func (h *DfuPlatformHandle) ControlTransfer(requestType, request uint8, value, index uint16, data []byte, timeout time.Duration) (int, error) {
	pkt := winusbSetupPacket{
		RequestType: requestType,
		Request:     request,
		Value:       value,
		Index:       index,
		Length:      uint16(len(data)),
	}

	// In x64 calling convention, an 8-byte struct is passed as a 64-bit value in RDX
	pktVal := *(*uintptr)(unsafe.Pointer(&pkt))

	var dataPtr unsafe.Pointer
	if len(data) > 0 {
		dataPtr = unsafe.Pointer(&data[0])
	}

	var transferred uint32
	r0, _, e1 := syscall.SyscallN(
		procWinUsb_ControlTransfer.Addr(),
		h.winusbHandle,
		pktVal,
		uintptr(dataPtr),
		uintptr(len(data)),
		uintptr(unsafe.Pointer(&transferred)),
		0,
	)

	if r0 == 0 {
		return 0, fmt.Errorf("WinUsb_ControlTransfer failed: %w", e1)
	}

	return int(transferred), nil
}

