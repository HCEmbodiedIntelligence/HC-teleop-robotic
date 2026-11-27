#include <pybind11/pybind11.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace py = pybind11;

namespace {

constexpr std::size_t kFecHeaderSize = 14;
constexpr std::size_t kShortMaskPackets = 16;

inline uint16_t read_be16(const uint8_t* p) {
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) |
                               static_cast<uint16_t>(p[1]));
}

inline uint32_t read_be32(const uint8_t* p) {
  return (static_cast<uint32_t>(p[0]) << 24) |
         (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) |
         static_cast<uint32_t>(p[3]);
}

inline void write_be16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}

inline void write_be32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>(v & 0xFF);
}

inline void xor_buffers(uint8_t* dst, const uint8_t* src, std::size_t len) {
  std::size_t i = 0;
  for (; i + 8 <= len; i += 8) {
    uint64_t a, b;
    std::memcpy(&a, dst + i, 8);
    std::memcpy(&b, src + i, 8);
    a ^= b;
    std::memcpy(dst + i, &a, 8);
  }
  for (; i < len; ++i) {
    dst[i] ^= src[i];
  }
}

struct BytesView {
  const uint8_t* data;
  std::size_t size;
};

BytesView require_bytes(const py::handle& obj, const char* err_msg) {
  if (!PyBytes_Check(obj.ptr())) {
    throw py::value_error(err_msg);
  }
  char* buf = nullptr;
  Py_ssize_t len = 0;
  if (PyBytes_AsStringAndSize(obj.ptr(), &buf, &len) != 0) {
    throw py::error_already_set();
  }
  return {reinterpret_cast<const uint8_t*>(buf), static_cast<std::size_t>(len)};
}

struct RtpLayoutRaw {
  uint16_t sequence_number;
  uint32_t ssrc;
  std::size_t header_size;
  std::size_t payload_end;
};

RtpLayoutRaw parse_rtp_layout(const uint8_t* data, std::size_t len) {
  if (len < 12 || (data[0] >> 6) != 2) {
    throw py::value_error("Expected a complete RTP v2 bytes packet");
  }
  std::size_t header_size = 12 + static_cast<std::size_t>(data[0] & 15) * 4;
  if (header_size > len) {
    throw py::value_error("Truncated RTP CSRC list");
  }
  if (data[0] & 0x10) {
    if (header_size + 4 > len) {
      throw py::value_error("Truncated RTP extension header");
    }
    const std::size_t words = read_be16(data + header_size + 2);
    header_size += 4 + words * 4;
    if (header_size > len) {
      throw py::value_error("Truncated RTP extension data");
    }
  }
  std::size_t payload_end = len;
  if (data[0] & 0x20) {
    const std::size_t padding_size = data[len - 1];
    if (padding_size == 0 || padding_size > len - header_size) {
      throw py::value_error("Invalid RTP padding");
    }
    payload_end -= padding_size;
  }
  return {read_be16(data + 2), read_be32(data + 8), header_size, payload_end};
}

struct ExtensionIdMask {
  std::array<bool, 256> mask{};
  bool any = false;
};

ExtensionIdMask parse_extension_ids(const py::iterable& ids) {
  ExtensionIdMask out;
  for (py::handle item : ids) {
    if (PyBool_Check(item.ptr()) || !PyLong_Check(item.ptr())) {
      throw py::value_error("Extension IDs must be integers in 1..255");
    }
    int overflow = 0;
    const long val = PyLong_AsLongAndOverflow(item.ptr(), &overflow);
    if (PyErr_Occurred()) {
      throw py::error_already_set();
    }
    if (overflow != 0 || val < 1 || val > 255) {
      throw py::value_error("Extension IDs must be integers in 1..255");
    }
    out.mask[static_cast<std::size_t>(val)] = true;
    out.any = true;
  }
  return out;
}

// Canonicalizes mutable extensions into `out` if modified; returns true if
// `out` was populated, false if original bytes are unchanged.
bool canonicalize_extensions_buf(const uint8_t* data,
                                 std::size_t len,
                                 const ExtensionIdMask& ids,
                                 std::vector<uint8_t>& out) {
  parse_rtp_layout(data, len);
  if (!ids.any || !(data[0] & 0x10)) {
    return false;
  }
  const std::size_t offset = 12 + static_cast<std::size_t>(data[0] & 15) * 4;
  const uint16_t profile = read_be16(data + offset);
  const std::size_t words = read_be16(data + offset + 2);
  std::size_t cursor = offset + 4;
  const std::size_t end = cursor + words * 4;

  out.assign(data, data + len);
  while (cursor < end) {
    const uint8_t element = data[cursor++];
    if (element == 0) {
      continue;
    }
    uint8_t extension_id = 0;
    std::size_t ext_len = 0;
    if (profile == 0xBEDE) {
      extension_id = element >> 4;
      if (extension_id == 15) {
        break;
      }
      ext_len = static_cast<std::size_t>(element & 15) + 1;
    } else if ((profile & 0xFFF0) == 0x1000) {
      extension_id = element;
      if (cursor >= end) {
        throw py::value_error("Truncated two-byte RTP extension element");
      }
      ext_len = data[cursor++];
    } else {
      throw py::value_error(
          "Cannot canonicalize unknown RTP extension profile");
    }
    if (cursor + ext_len > end) {
      throw py::value_error("Truncated RTP extension element data");
    }
    if (ids.mask[extension_id]) {
      std::memset(out.data() + cursor, 0, ext_len);
    }
    cursor += ext_len;
  }
  return true;
}

py::tuple rtp_layout_tuple(py::object packet) {
  const auto view =
      require_bytes(packet, "Expected a complete RTP v2 bytes packet");
  const auto layout = parse_rtp_layout(view.data, view.size);
  return py::make_tuple(layout.sequence_number, layout.ssrc, layout.header_size,
                        layout.payload_end);
}

py::bytes zero_mutable_extensions_py(py::object packet,
                                     py::iterable extension_ids) {
  const auto view =
      require_bytes(packet, "Expected a complete RTP v2 bytes packet");
  const auto ids = parse_extension_ids(extension_ids);
  std::vector<uint8_t> buf;
  if (!canonicalize_extensions_buf(view.data, view.size, ids, buf)) {
    return py::reinterpret_borrow<py::bytes>(packet);
  }
  return py::bytes(reinterpret_cast<const char*>(buf.data()), buf.size());
}

py::bytes encode_ulpfec_py(py::sequence packets,
                           py::iterable mutable_extension_ids,
                           py::object max_payload_size_obj) {
  const std::size_t count = static_cast<std::size_t>(py::len(packets));
  if (count < 1 || count > kShortMaskPackets) {
    throw py::value_error("A short FEC mask protects 1..16 packets");
  }
  const auto ids = parse_extension_ids(mutable_extension_ids);

  std::vector<std::vector<uint8_t>> owned_bufs;
  owned_bufs.reserve(count);
  std::vector<BytesView> views;
  views.reserve(count);
  std::vector<RtpLayoutRaw> layouts;
  layouts.reserve(count);

  std::array<bool, 65536> seq_seen{};
  std::vector<uint16_t> sequences;
  sequences.reserve(count);
  uint32_t shared_ssrc = 0;

  for (std::size_t i = 0; i < count; ++i) {
    py::object item = packets[i];
    const auto raw =
        require_bytes(item, "Expected a complete RTP v2 bytes packet");
    std::vector<uint8_t> normalized;
    BytesView effective = raw;
    if (canonicalize_extensions_buf(raw.data, raw.size, ids, normalized)) {
      owned_bufs.push_back(std::move(normalized));
      effective = {owned_bufs.back().data(), owned_bufs.back().size()};
    }
    const auto layout = parse_rtp_layout(effective.data, effective.size);
    if (seq_seen[layout.sequence_number]) {
      throw py::value_error("Duplicate protected RTP sequence number");
    }
    seq_seen[layout.sequence_number] = true;
    if (i == 0) {
      shared_ssrc = layout.ssrc;
    } else if (layout.ssrc != shared_ssrc) {
      throw py::value_error("Protected RTP packets must share one SSRC");
    }
    sequences.push_back(layout.sequence_number);
    views.push_back(effective);
    layouts.push_back(layout);
  }

  bool found_base = false;
  uint16_t base = 0;
  for (uint16_t candidate : sequences) {
    bool ok = true;
    for (uint16_t seq : sequences) {
      if (static_cast<uint16_t>(seq - candidate) >= 16) {
        ok = false;
        break;
      }
    }
    if (ok) {
      base = candidate;
      found_base = true;
      break;
    }
  }
  if (!found_base) {
    throw py::value_error("Protected sequence-number span exceeds short mask");
  }

  std::size_t protection_length = 0;
  for (const auto& v : views) {
    protection_length = std::max(protection_length, v.size - 12);
  }
  if (protection_length > 65535) {
    throw py::value_error("RTP tail exceeds ULPFEC 16-bit protection length");
  }
  if (!max_payload_size_obj.is_none()) {
    const std::size_t max_payload_size = max_payload_size_obj.cast<std::size_t>();
    if (kFecHeaderSize + protection_length > max_payload_size) {
      throw py::value_error("FEC payload exceeds caller MTU budget");
    }
  }

  std::vector<uint8_t> out(kFecHeaderSize + protection_length, 0);
  uint8_t first = 0;
  uint8_t second = 0;
  uint32_t timestamp = 0;
  uint16_t length = 0;
  uint16_t mask = 0;

  uint8_t* parity = out.data() + kFecHeaderSize;
  for (std::size_t i = 0; i < count; ++i) {
    const auto& v = views[i];
    const auto& l = layouts[i];
    first ^= (v.data[0] & 0x3F);
    second ^= v.data[1];
    timestamp ^= read_be32(v.data + 4);
    length ^= static_cast<uint16_t>(v.size - 12);
    const uint16_t offset = static_cast<uint16_t>(l.sequence_number - base);
    mask |= static_cast<uint16_t>(1U << (15U - offset));
    xor_buffers(parity, v.data + 12, v.size - 12);
  }

  out[0] = first;
  out[1] = second;
  write_be16(out.data() + 2, base);
  write_be32(out.data() + 4, timestamp);
  write_be16(out.data() + 8, length);
  write_be16(out.data() + 10, static_cast<uint16_t>(protection_length));
  write_be16(out.data() + 12, mask);

  return py::bytes(reinterpret_cast<const char*>(out.data()), out.size());
}

std::vector<uint16_t> parse_protected_sequences(const uint8_t* data,
                                                std::size_t len,
                                                uint16_t& protection_length) {
  if (len < kFecHeaderSize) {
    throw py::value_error("Truncated ULPFEC payload");
  }
  if (data[0] & 0xC0) {
    throw py::value_error("Only E=0/L=0 ULPFEC payloads are supported");
  }
  const uint16_t base = read_be16(data + 2);
  protection_length = read_be16(data + 10);
  const uint16_t mask = read_be16(data + 12);
  if (len != kFecHeaderSize + static_cast<std::size_t>(protection_length)) {
    throw py::value_error("Truncated or multi-level ULPFEC payload");
  }
  if ((mask & 0x8000U) == 0) {
    throw py::value_error("Level-0-only FEC mask must include sequence base");
  }
  std::vector<uint16_t> seqs;
  seqs.reserve(16);
  for (unsigned idx = 0; idx < 16; ++idx) {
    if (mask & (1U << (15U - idx))) {
      seqs.push_back(static_cast<uint16_t>(base + idx));
    }
  }
  return seqs;
}

py::tuple protected_sequences_py(py::object fec_payload) {
  const auto view = require_bytes(fec_payload, "Truncated ULPFEC payload");
  uint16_t protection_length = 0;
  const auto seqs = parse_protected_sequences(view.data, view.size,
                                              protection_length);
  py::tuple result(seqs.size());
  for (std::size_t i = 0; i < seqs.size(); ++i) {
    result[i] = py::int_(seqs[i]);
  }
  return result;
}

py::object recover_ulpfec_py(py::object fec_payload,
                             py::object received_packets,
                             py::object ssrc_obj,
                             py::iterable mutable_extension_ids) {
  if (PyBool_Check(ssrc_obj.ptr()) || !PyLong_Check(ssrc_obj.ptr())) {
    throw py::value_error("SSRC must be an unsigned 32-bit integer");
  }
  int overflow = 0;
  const long long ssrc_ll =
      PyLong_AsLongLongAndOverflow(ssrc_obj.ptr(), &overflow);
  if (PyErr_Occurred()) {
    throw py::error_already_set();
  }
  if (overflow != 0 || ssrc_ll < 0 || ssrc_ll > 0xFFFFFFFFLL) {
    throw py::value_error("SSRC must be an unsigned 32-bit integer");
  }
  const uint32_t ssrc = static_cast<uint32_t>(ssrc_ll);

  const auto fec_view = require_bytes(fec_payload, "Truncated ULPFEC payload");
  uint16_t protection_length = 0;
  const auto protected_seqs =
      parse_protected_sequences(fec_view.data, fec_view.size, protection_length);

  std::unordered_set<uint16_t> protected_set(protected_seqs.begin(),
                                             protected_seqs.end());
  const auto ids = parse_extension_ids(mutable_extension_ids);

  py::object iterable = received_packets;
  if (PyMapping_Check(received_packets.ptr()) &&
      py::hasattr(received_packets, "values")) {
    iterable = received_packets.attr("values")();
  }

  std::unordered_map<uint16_t, std::vector<uint8_t>> received;
  received.reserve(protected_seqs.size());

  for (py::handle item : iterable) {
    const auto raw =
        require_bytes(item, "Expected a complete RTP v2 bytes packet");
    const auto layout = parse_rtp_layout(raw.data, raw.size);
    if (protected_set.count(layout.sequence_number) == 0) {
      continue;
    }
    if (layout.ssrc != ssrc) {
      throw py::value_error("Received protected packet has wrong SSRC");
    }
    std::vector<uint8_t> normalized;
    if (!canonicalize_extensions_buf(raw.data, raw.size, ids, normalized)) {
      normalized.assign(raw.data, raw.data + raw.size);
    }
    auto it = received.find(layout.sequence_number);
    if (it != received.end()) {
      if (it->second != normalized) {
        throw py::value_error("Conflicting duplicate protected RTP packet");
      }
    } else {
      received.emplace(layout.sequence_number, std::move(normalized));
    }
  }

  std::vector<uint16_t> missing;
  for (uint16_t seq : protected_seqs) {
    if (received.count(seq) == 0) {
      missing.push_back(seq);
    }
  }
  if (missing.size() != 1) {
    return py::none();
  }

  uint8_t first = fec_view.data[0];
  uint8_t second = fec_view.data[1];
  uint32_t timestamp = read_be32(fec_view.data + 4);
  uint16_t length = read_be16(fec_view.data + 8);
  std::vector<uint8_t> parity(fec_view.data + kFecHeaderSize,
                              fec_view.data + fec_view.size);

  for (const auto& [seq, pkt] : received) {
    (void)seq;
    if (pkt.size() - 12 > protection_length) {
      throw py::value_error("Received RTP exceeds FEC protection length");
    }
    first ^= (pkt[0] & 0x3F);
    second ^= pkt[1];
    timestamp ^= read_be32(pkt.data() + 4);
    length ^= static_cast<uint16_t>(pkt.size() - 12);
    xor_buffers(parity.data(), pkt.data() + 12, pkt.size() - 12);
  }
  if (length > protection_length) {
    throw py::value_error("FEC does not fully protect recovered RTP length");
  }

  std::vector<uint8_t> recovered(12 + static_cast<std::size_t>(length));
  recovered[0] = static_cast<uint8_t>(0x80U | first);
  recovered[1] = second;
  write_be16(recovered.data() + 2, missing[0]);
  write_be32(recovered.data() + 4, timestamp);
  write_be32(recovered.data() + 8, ssrc);
  if (length > 0) {
    std::memcpy(recovered.data() + 12, parity.data(), length);
  }
  parse_rtp_layout(recovered.data(), recovered.size());
  return py::bytes(reinterpret_cast<const char*>(recovered.data()),
                   recovered.size());
}

uint8_t require_red_pt(py::object pt_obj) {
  if (PyBool_Check(pt_obj.ptr()) || !PyLong_Check(pt_obj.ptr())) {
    throw py::value_error("RED block payload type must be in 0..127");
  }
  int overflow = 0;
  const long val = PyLong_AsLongAndOverflow(pt_obj.ptr(), &overflow);
  if (PyErr_Occurred()) {
    throw py::error_already_set();
  }
  if (overflow != 0 || val < 0 || val > 127) {
    throw py::value_error("RED block payload type must be in 0..127");
  }
  return static_cast<uint8_t>(val);
}

py::bytes wrap_red_payload_py(py::object payload, py::object block_pt_obj) {
  const uint8_t pt = require_red_pt(block_pt_obj);
  const auto view = require_bytes(payload, "RED payload must be bytes");
  std::string out(1 + view.size, '\0');
  out[0] = static_cast<char>(pt);
  if (view.size > 0) {
    std::memcpy(out.data() + 1, view.data, view.size);
  }
  return py::bytes(out);
}

py::tuple unwrap_red_payload_py(py::object payload) {
  const auto view =
      require_bytes(payload, "Truncated RED primary block header");
  if (view.size == 0) {
    throw py::value_error("Truncated RED primary block header");
  }
  if (view.data[0] & 0x80) {
    throw py::value_error("Only single-primary RED blocks are supported");
  }
  return py::make_tuple(
      py::int_(view.data[0]),
      py::bytes(reinterpret_cast<const char*>(view.data + 1), view.size - 1));
}

py::bytes red_wrap_py(py::object packet, py::object red_pt_obj) {
  const auto view =
      require_bytes(packet, "Expected a complete RTP v2 bytes packet");
  const auto layout = parse_rtp_layout(view.data, view.size);
  const uint8_t red_pt = require_red_pt(red_pt_obj);
  const uint8_t orig_pt = view.data[1] & 0x7F;
  const std::size_t payload_len = layout.payload_end - layout.header_size;
  const std::size_t tail_len = view.size - layout.payload_end;

  std::string out(view.size + 1, '\0');
  uint8_t* dst = reinterpret_cast<uint8_t*>(out.data());
  std::memcpy(dst, view.data, layout.header_size);
  dst[1] = static_cast<uint8_t>((dst[1] & 0x80U) | red_pt);
  dst[layout.header_size] = orig_pt;
  if (payload_len > 0) {
    std::memcpy(dst + layout.header_size + 1, view.data + layout.header_size,
                payload_len);
  }
  if (tail_len > 0) {
    std::memcpy(dst + layout.header_size + 1 + payload_len,
                view.data + layout.payload_end, tail_len);
  }
  return py::bytes(out);
}

py::bytes red_unwrap_py(py::object packet) {
  const auto view =
      require_bytes(packet, "Expected a complete RTP v2 bytes packet");
  const auto layout = parse_rtp_layout(view.data, view.size);
  if (layout.payload_end <= layout.header_size) {
    throw py::value_error("Truncated RED primary block header");
  }
  const uint8_t block_pt = view.data[layout.header_size];
  if (block_pt & 0x80) {
    throw py::value_error("Only single-primary RED blocks are supported");
  }
  const std::size_t inner_len = layout.payload_end - layout.header_size - 1;
  const std::size_t tail_len = view.size - layout.payload_end;

  std::string out(view.size - 1, '\0');
  uint8_t* dst = reinterpret_cast<uint8_t*>(out.data());
  std::memcpy(dst, view.data, layout.header_size);
  dst[1] = static_cast<uint8_t>((dst[1] & 0x80U) | block_pt);
  if (inner_len > 0) {
    std::memcpy(dst + layout.header_size, view.data + layout.header_size + 1,
                inner_len);
  }
  if (tail_len > 0) {
    std::memcpy(dst + layout.header_size + inner_len,
                view.data + layout.payload_end, tail_len);
  }
  return py::bytes(out);
}

}  // namespace

PYBIND11_MODULE(_rtp_fec_cpp, m) {
  m.doc() = "C++17 accelerated RFC 5109 ULPFEC and RFC 2198 RED wire codec";

  m.def("rtp_layout_tuple", &rtp_layout_tuple, py::arg("packet"));
  m.def("zero_mutable_extensions", &zero_mutable_extensions_py,
        py::arg("packet"), py::arg("extension_ids") = py::tuple());
  m.def("encode_ulpfec", &encode_ulpfec_py, py::arg("packets"),
        py::arg("mutable_extension_ids") = py::tuple(),
        py::arg("max_payload_size") = py::none());
  m.def("protected_sequences", &protected_sequences_py, py::arg("fec_payload"));
  m.def("recover_ulpfec", &recover_ulpfec_py, py::arg("fec_payload"),
        py::arg("received_packets"), py::arg("ssrc"),
        py::arg("mutable_extension_ids") = py::tuple());
  m.def("wrap_red_payload", &wrap_red_payload_py, py::arg("payload"),
        py::arg("block_payload_type"));
  m.def("unwrap_red_payload", &unwrap_red_payload_py, py::arg("payload"));
  m.def("red_wrap", &red_wrap_py, py::arg("packet"),
        py::arg("red_payload_type"));
  m.def("red_unwrap", &red_unwrap_py, py::arg("packet"));
}

