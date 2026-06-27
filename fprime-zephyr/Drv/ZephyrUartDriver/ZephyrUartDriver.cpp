// ======================================================================
// \title  ZephyrUartDriver.cpp
// \author ethanchee
// \brief  cpp file for ZephyrUartDriver component implementation class
// ======================================================================

#include "fprime-zephyr/Drv/ZephyrUartDriver/ZephyrUartDriver.hpp"
#include <Fw/FPrimeBasicTypes.hpp>
#include <zephyr/kernel.h>

namespace Zephyr {

// ----------------------------------------------------------------------
// Construction, initialization, and destruction
// ----------------------------------------------------------------------

ZephyrUartDriver ::ZephyrUartDriver(const char* const compName) : ZephyrUartDriverComponentBase(compName) {}

ZephyrUartDriver ::~ZephyrUartDriver() {}

void ZephyrUartDriver::configure(const struct device* dev, U32 baud_rate) {
    FW_ASSERT(dev != nullptr);
    m_dev = dev;

    if (!device_is_ready(this->m_dev)) {
        return;
    }

    struct uart_config uart_cfg;
    int err = uart_config_get(this->m_dev, &uart_cfg);
    if (err == 0) {
        // Keep device tree defaults (including hw-flow-control)
        uart_cfg.baudrate = baud_rate;
    } else {
        // Fallback if config_get is unsupported
        uart_cfg.baudrate = baud_rate;
        uart_cfg.parity = UART_CFG_PARITY_NONE;
        uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
        uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
        uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    }
    uart_configure(this->m_dev, &uart_cfg);

    ring_buf_init(&this->m_ring_buf, RING_BUF_SIZE, this->m_ring_buf_data);

    uart_irq_callback_user_data_set(this->m_dev, serial_cb, this);
    uart_irq_rx_enable(this->m_dev);

    if (this->isConnected_ready_OutputPort(0)) {
        this->ready_out(0);
    }
}

void ZephyrUartDriver::serial_cb(const struct device* dev, void* user_data) {
    ZephyrUartDriver* driver = static_cast<ZephyrUartDriver*>(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        U8 buf[64];
        int recv_len = uart_fifo_read(dev, buf, sizeof(buf));
        if (recv_len > 0) {
            ring_buf_put(&driver->m_ring_buf, buf, recv_len);
        }
    }
}

// ----------------------------------------------------------------------
// Handler implementations for user-defined typed input ports
// ----------------------------------------------------------------------

void ZephyrUartDriver ::schedIn_handler(const FwIndexType portNum, U32 context) {
    Fw::Buffer recv_buffer = this->allocate_out(0, SERIAL_BUFFER_SIZE);

    U32 recv_size = ring_buf_get(&this->m_ring_buf, recv_buffer.getData(), recv_buffer.getSize());
    if (recv_size == 0) {
        // No data received, deallocate buffer
        this->deallocate_out(0, recv_buffer);
    } else {
        recv_buffer.setSize(recv_size);
        recv_out(0, recv_buffer, Drv::ByteStreamStatus::OP_OK);
    }
}

Drv::ByteStreamStatus ZephyrUartDriver ::send_handler(const FwIndexType portNum, Fw::Buffer& sendBuffer) {
    for (U32 i = 0; i < sendBuffer.getSize(); i++) {
        uart_poll_out(this->m_dev, sendBuffer.getData()[i]);
    }
    return Drv::ByteStreamStatus::OP_OK;
}

    Drv::ByteStreamStatus ZephyrUartDriver ::
        send_handler(
            const FwIndexType portNum,
            Fw::Buffer &sendBuffer
        )
    {
        U32 size = sendBuffer.getSize();
        
        // Transmit byte by byte
        for (U32 i = 0; i < size; i++) {
            uart_poll_out(this->m_dev, sendBuffer.getData()[i]);
        }
        
        // Inter-packet delay: Prevents back-to-back packets from merging
        // in the Linux UART buffer on the RPi side. Without this delay,
        // LinuxUartDriver with VMIN=0 will read multiple packets in one
        // read() call, causing GenericHub to see corrupted data.
        // 5ms is safe for 31-byte packets at 115200 baud (~2.7ms transmission time)
        k_sleep(K_MSEC(5));
        
        return Drv::ByteStreamStatus::OP_OK;
    }

}  // end namespace Zephyr
