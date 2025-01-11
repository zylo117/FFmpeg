 /**************************************************************************************************
 *
 * Copyright (c) 2019-2024 Axera Semiconductor Co., Ltd. All Rights Reserved.
 *
 * This source file is the property of Axera Semiconductor Co., Ltd. and
 * may not be copied or distributed in any isomorphic form without the prior
 * written consent of Axera Semiconductor Co., Ltd.
 *
 **************************************************************************************************/

#include "axcl_dma_buffer.h"
#include <error.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define AX_MM_DEV "/dev/ax_mmb_dev"
#define TAG "pcie"

// ioctl cmd
#define AX_IOC_PCIe_BASE 'H'
#define AX_IOC_PCIE_DMA_GET_PHY_BASE    _IOW(AX_IOC_PCIe_BASE, 3, unsigned int)
#define AX_IOC_PCIe_ALLOC_MEMORY        _IOW(AX_IOC_PCIe_BASE, 4, unsigned int)
#define AX_IOC_PCIe_ALLOC_MEMCACHED     _IOW(AX_IOC_PCIe_BASE, 5, unsigned int)
#define AX_IOC_PCIe_FLUSH_CACHED        _IOW(AX_IOC_PCIe_BASE, 6, struct ax_mem_info)
#define AX_IOC_PCIe_INVALID_CACHED      _IOW(AX_IOC_PCIe_BASE, 7, struct ax_mem_info)

#define LOG_MM_E(tag, format, ...) \
    do { \
        std::cerr << "[" << tag << "] "; \
        fprintf(stderr, format, ##__VA_ARGS__); \
        std::cerr << std::endl; \
    } while (0)

struct ax_mem_info {
    uint64_t phy;
    uint64_t vir;
    uint64_t size;
};

dma_buffer::dma_buffer() : m_fd(-1) {
}

dma_buffer::~dma_buffer() {
    free();
}

dma_buffer::dma_buffer(dma_buffer &&rhs) noexcept {
    m_fd = std::__exchange(rhs.m_fd, -1);
    m_mem = std::__exchange(rhs.m_mem, dma_mem{});
}

dma_buffer &dma_buffer::operator=(dma_buffer &&rhs) noexcept {
    m_fd = std::__exchange(rhs.m_fd, -1);
    m_mem = std::__exchange(rhs.m_mem, dma_mem{});

    return *this;
}

bool dma_buffer::alloc(size_t size, bool cached) {
    if (m_fd > 0) {
        LOG_MM_E(TAG, "{} is opened", AX_MM_DEV);
        return false;
    }

    int fd = ::open(AX_MM_DEV, O_RDWR);
    if (fd < 0) {
        LOG_MM_E(TAG, "open {} fail", AX_MM_DEV);
        return false;
    }

#ifndef AXCL_CMA_CACHED
    cached = false;
#endif

    int ret;
    uint64_t dma_buffer_size = size;
    if (cached) {
        if (ret = ::ioctl(fd, AX_IOC_PCIe_ALLOC_MEMCACHED, &dma_buffer_size); ret < 0) {
            LOG_MM_E(TAG, "allocate pcie cached dma buffer (size: {}) fail, {}", dma_buffer_size, ::strerror(errno));
            ::close(fd);
            return false;
        }
    } else {
        if (ret = ::ioctl(fd, AX_IOC_PCIe_ALLOC_MEMORY, &dma_buffer_size); ret < 0) {
            LOG_MM_E(TAG, "allocate pcie dma buffer (size: {}) fail, {}", dma_buffer_size, ::strerror(errno));
            ::close(fd);
            return false;
        }
    }

    uint64_t phy;
    if (ret = ::ioctl(fd, AX_IOC_PCIE_DMA_GET_PHY_BASE, &phy); ret < 0) {
        LOG_MM_E(TAG, "get pcie dma phy address fail, {}", ::strerror(errno));
        ::close(fd);
        return false;
    }

    void *vir;
    if (vir = ::mmap(NULL, dma_buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0); !vir) {
        LOG_MM_E(TAG, "mmap pcie dma phy address fail, {}", ::strerror(errno));
        ::close(fd);
        return false;
    }

    m_fd = fd;
    m_mem.phy = phy;
    m_mem.vir = vir;
    m_mem.size = dma_buffer_size;
    m_mem.cached = cached;
    if (cached) {
        m_mem.flush = std::bind(&dma_buffer::flush, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        m_mem.invalidate = std::bind(&dma_buffer::invalidate, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    }

    return true;
}

void dma_buffer::free() {
    if (m_fd > 0) {
        if (m_mem.vir) {
            ::munmap(m_mem.vir, m_mem.size);
            m_mem.vir = nullptr;
        }

        ::close(m_fd);
        m_fd = -1;
    }

    m_mem.phy = 0;
    m_mem.size = 0;
}

bool dma_buffer::flush(uint64_t phy, void *vir, uint32_t size) {
    struct ax_mem_info mem = {phy, reinterpret_cast<uint64_t>(vir), size};
    if (int ret = ::ioctl(m_fd, AX_IOC_PCIe_FLUSH_CACHED, &mem); ret < 0) {
        LOG_MM_E(TAG, "flush dma buffer (phy {} vir {} size {}) fail, {}", phy, vir, size, ::strerror(errno));
        return false;
    }

    return true;
}

bool dma_buffer::invalidate(uint64_t phy, void *vir, uint32_t size) {
    struct ax_mem_info mem = {phy, reinterpret_cast<uint64_t>(vir), size};
    if (int ret = ::ioctl(m_fd, AX_IOC_PCIe_INVALID_CACHED, &mem); ret < 0) {
        LOG_MM_E(TAG, "invalidate dma buffer (phy {} vir {} size {}) fail, {}", phy, vir, size, ::strerror(errno));
        return false;
    }

    return true;
}

extern "C" {
void* dma_buffer_create() {
    return new dma_buffer();
}

void dma_buffer_destroy(dma_buffer* db) {
    delete db;
}

bool dma_buffer_alloc(dma_buffer* db, size_t size, bool cached) {
    return db->alloc(size, cached);
}

void dma_buffer_free(dma_buffer* db) {
    db->free();
}

bool dma_buffer_flush(dma_buffer* db, uint64_t phy, void* vir, uint32_t size) {
    return db->flush(phy, vir, size);
}

bool dma_buffer_invalidate(dma_buffer* db, uint64_t phy, void* vir, uint32_t size) {
    return db->invalidate(phy, vir, size);
}

const struct dma_mem* dma_buffer_get(dma_buffer* db) {
    return &db->get();
}
}
