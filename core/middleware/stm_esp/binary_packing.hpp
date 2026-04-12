#pragma once

#include <cstddef>
#include <cstdint>

namespace stm_esp::binary_packing
{
  class writer
  {
  public:
    writer(std::uint8_t *buffer, std::size_t capacity)
      : m_buffer(buffer)
      , m_capacity(capacity)
    {
    }

    bool write_bool(bool value) { return write_u8(value ? 1U : 0U); }
    bool write_u8(std::uint8_t value) { return write_value<std::uint8_t>(value); }
    bool write_u16(std::uint16_t value) { return write_value<std::uint16_t>(value); }
    bool write_u32(std::uint32_t value) { return write_value<std::uint32_t>(value); }
    bool write_u64(std::uint64_t value) { return write_value<std::uint64_t>(value); }
    bool write_i16(std::int16_t value) { return write_value<std::uint16_t>(static_cast<std::uint16_t>(value)); }
    bool write_i32(std::int32_t value) { return write_value<std::uint32_t>(static_cast<std::uint32_t>(value)); }
    bool write_i64(std::int64_t value) { return write_value<std::uint64_t>(static_cast<std::uint64_t>(value)); }
    std::size_t length(void) const { return m_length; }

  private:
    template <typename value_type>
    bool write_value(value_type value)
    {
      if (m_buffer == nullptr || (m_length + sizeof(value_type)) > m_capacity)
      {
        return false;
      }

      for (std::size_t byte_index = 0; byte_index < sizeof(value_type); ++byte_index)
      {
        m_buffer[m_length++] = static_cast<std::uint8_t>((value >> (byte_index * 8U)) & 0xFFU);
      }

      return true;
    }

    std::uint8_t *m_buffer = nullptr;
    std::size_t m_capacity = 0U;
    std::size_t m_length = 0U;
  };

  class reader
  {
  public:
    reader(const std::uint8_t *buffer, std::size_t length)
      : m_buffer(buffer)
      , m_length(length)
    {
    }

    bool read_bool(bool &value_out)
    {
      std::uint8_t value = 0U;

      if (!read_u8(value))
      {
        return false;
      }

      value_out = (value != 0U);
      return true;
    }

    bool read_u8(std::uint8_t &value_out) { return read_value<std::uint8_t>(value_out); }
    bool read_u16(std::uint16_t &value_out) { return read_value<std::uint16_t>(value_out); }
    bool read_u32(std::uint32_t &value_out) { return read_value<std::uint32_t>(value_out); }
    bool read_u64(std::uint64_t &value_out) { return read_value<std::uint64_t>(value_out); }

    bool read_i16(std::int16_t &value_out)
    {
      std::uint16_t value = 0U;

      if (!read_u16(value))
      {
        return false;
      }

      value_out = static_cast<std::int16_t>(value);
      return true;
    }

    bool read_i32(std::int32_t &value_out)
    {
      std::uint32_t value = 0U;

      if (!read_u32(value))
      {
        return false;
      }

      value_out = static_cast<std::int32_t>(value);
      return true;
    }

    bool read_i64(std::int64_t &value_out)
    {
      std::uint64_t value = 0U;

      if (!read_u64(value))
      {
        return false;
      }

      value_out = static_cast<std::int64_t>(value);
      return true;
    }

    bool is_finished(void) const { return m_offset == m_length; }

  private:
    template <typename value_type>
    bool read_value(value_type &value_out)
    {
      if (m_buffer == nullptr || (m_offset + sizeof(value_type)) > m_length)
      {
        return false;
      }

      value_type value = 0;

      for (std::size_t byte_index = 0; byte_index < sizeof(value_type); ++byte_index)
      {
        value |= static_cast<value_type>(m_buffer[m_offset + byte_index]) << (byte_index * 8U);
      }

      m_offset += sizeof(value_type);
      value_out = value;
      return true;
    }

    const std::uint8_t *m_buffer = nullptr;
    std::size_t m_length = 0U;
    std::size_t m_offset = 0U;
  };
}
