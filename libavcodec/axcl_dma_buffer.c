#include "axcl_dma_buffer.h"
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

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
        fprintf(stderr, "[%s] ", tag); \
        fprintf(stderr, format, ##__VA_ARGS__); \
        fprintf(stderr, "\n"); \
    } while (0)

struct ax_mem_info {
    uint64_t phy;
    uint64_t vir;
    uint64_t size;
};


void dma_buffer_init(struct dma_buffer *buf) {
    buf->m_fd = -1;
}

void dma_buffer_destroy(struct dma_buffer *buf) {
    dma_buffer_free(buf);
}



int dma_buffer_alloc(struct dma_buffer *buf, size_t size) {
    if (buf->m_fd > 0) {
        LOG_MM_E(TAG, "%s is opened", AX_MM_DEV);
        return 0;
    }

    int fd = open(AX_MM_DEV, O_RDWR);
    if (fd < 0) {
        LOG_MM_E(TAG, "open %s fail", AX_MM_DEV);
        return 0;
    }

    int ret;
    uint64_t dma_buffer_size = size;
    ret = ioctl(fd, AX_IOC_PCIe_ALLOC_MEMORY, &dma_buffer_size);
    if (ret < 0) {
        LOG_MM_E(TAG, "allocate pcie dma buffer (size: %lu) fail, %d", dma_buffer_size, ret);
        close(fd);
        return 0;
    }

    uint64_t phy;
    ret = ioctl(fd, AX_IOC_PCIE_DMA_GET_PHY_BASE, &phy);
    if (ret < 0) {
        LOG_MM_E(TAG, "get pcie dma phy address fail, %d", ret);
        close(fd);
        return 0;
    }

    void *vir;
    if (!(vir = mmap(NULL, dma_buffer_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0))) {
        LOG_MM_E(TAG, "mmap pcie dma phy address fail, %d", ret);
        close(fd);
        return 0;
    }

    buf->m_fd = fd;
    buf->m_mem.phy = phy;
    buf->m_mem.vir = vir;
    buf->m_mem.size = dma_buffer_size;

    return 1;
}

void dma_buffer_free(struct dma_buffer *buf) {
    if (buf->m_fd > 0) {
        if (buf->m_mem.vir) {
            munmap(buf->m_mem.vir, buf->m_mem.size);
            buf->m_mem.vir = NULL;
        }

        close(buf->m_fd);
        buf->m_fd = -1;
    }

    buf->m_mem.phy = 0;
    buf->m_mem.size = 0;
}

const struct dma_mem *dma_buffer_get(const struct dma_buffer *buf) {
    return &buf->m_mem;
}