#include <stdint.h>
#include <stddef.h>

struct dma_mem {
    uint64_t phy;
    void     *vir;
    uint32_t size;
};

struct dma_buffer {
    int32_t        m_fd;
    struct dma_mem m_mem;
};

void dma_buffer_init(struct dma_buffer *buf);

void dma_buffer_destroy(struct dma_buffer *buf);

int dma_buffer_alloc(struct dma_buffer *buf, size_t size);

void dma_buffer_free(struct dma_buffer *buf);

const struct dma_mem *dma_buffer_get(const struct dma_buffer *buf);