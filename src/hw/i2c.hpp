
#include <FreeRTOS.h>
#include <semphr.h>
#include <i2c_dma.h>

namespace sparkie
{
    class I2C
    {
        public:
            static I2C* getInstance();

            void init();

            int raw_write(uint8_t addr, const uint8_t *wbuf, size_t wbuf_len);
            int write_into(uint8_t addr, uint8_t reg, const uint8_t *wbuf, size_t wbuf_len);
            int write_byte(uint8_t addr, uint8_t reg, uint8_t byte);
            int write_word(uint8_t addr, uint8_t reg, uint16_t word);

            int raw_read(uint8_t addr, uint8_t *rbuf, size_t rbuf_len);
            int read_byte(uint8_t addr, uint8_t reg, uint8_t *data);
            int read_from(uint8_t addr, uint8_t reg, uint8_t *rbuf, size_t rbuf_len);
            int read_word_swapped(uint8_t addr, uint8_t reg, uint16_t *word);

        private:
            i2c_dma_t *i2c_dma;
            static I2C* instance;
    };
} // namespace sparkie   