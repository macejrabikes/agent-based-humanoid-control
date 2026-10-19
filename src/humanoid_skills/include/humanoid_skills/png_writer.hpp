// png_writer.hpp - zapis RGB8 obrazka do PNG bez externych kniznic (nekomprimovane deflate bloky).
#pragma once
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace pngw
{
inline uint32_t crc32(const uint8_t * d, size_t n, uint32_t c = 0)
{
  static uint32_t table[256];
  static bool init = false;
  if (!init) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t x = i;
      for (int k = 0; k < 8; ++k) {x = (x & 1) ? 0xEDB88320u ^ (x >> 1) : x >> 1;}
      table[i] = x;
    }
    init = true;
  }
  c = ~c;
  for (size_t i = 0; i < n; ++i) {c = table[(c ^ d[i]) & 0xFF] ^ (c >> 8);}
  return ~c;
}

inline void put32(std::vector<uint8_t> & v, uint32_t x)
{
  v.push_back(x >> 24); v.push_back(x >> 16); v.push_back(x >> 8); v.push_back(x);
}

inline void chunk(std::vector<uint8_t> & out, const char * type, const std::vector<uint8_t> & data)
{
  put32(out, static_cast<uint32_t>(data.size()));
  std::vector<uint8_t> td(type, type + 4);
  td.insert(td.end(), data.begin(), data.end());
  out.insert(out.end(), td.begin(), td.end());
  put32(out, crc32(td.data(), td.size()));
}

// rgb: width*height*3 bajtov, riadok po riadku; step = bajtov na riadok
inline bool write_rgb(const std::string & path, const uint8_t * rgb, uint32_t w, uint32_t h, uint32_t step)
{
  std::vector<uint8_t> raw;
  raw.reserve(static_cast<size_t>(h) * (w * 3 + 1));
  for (uint32_t y = 0; y < h; ++y) {
    raw.push_back(0);   // filter none
    raw.insert(raw.end(), rgb + static_cast<size_t>(y) * step, rgb + static_cast<size_t>(y) * step + w * 3);
  }
  std::vector<uint8_t> z = {0x78, 0x01};
  size_t pos = 0;
  while (pos < raw.size()) {
    const size_t n = std::min<size_t>(65535, raw.size() - pos);
    z.push_back(pos + n >= raw.size() ? 1 : 0);
    z.push_back(n & 0xFF); z.push_back(n >> 8);
    z.push_back(~n & 0xFF); z.push_back((~n >> 8) & 0xFF);
    z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + n);
    pos += n;
  }
  uint32_t a = 1, b = 0;
  for (uint8_t c : raw) {a = (a + c) % 65521; b = (b + a) % 65521;}
  put32(z, (b << 16) | a);

  std::vector<uint8_t> out = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  std::vector<uint8_t> ihdr;
  put32(ihdr, w); put32(ihdr, h);
  ihdr.push_back(8); ihdr.push_back(2); ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
  chunk(out, "IHDR", ihdr);
  chunk(out, "IDAT", z);
  chunk(out, "IEND", {});
  FILE * f = std::fopen(path.c_str(), "wb");
  if (!f) {return false;}
  const bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
  std::fclose(f);
  return ok;
}
}  // namespace pngw
