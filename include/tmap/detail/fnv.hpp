// BSD 3-Clause License
//
// Copyright (c) 2018, Gregory Meyer
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//	 notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//	 notice, this list of conditions and the following disclaimer in
//	 the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//	 from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef TMAP_DETAIL_FNV_HPP
#define TMAP_DETAIL_FNV_HPP

#include <climits>
#include <cstddef>

#include <string>
#include <string_view>
#include <typeindex>

namespace tmap::detail {
inline namespace literals {

constexpr std::size_t operator""_usize(unsigned long long x) noexcept {
	return static_cast<std::size_t>(x);
}

} // inline namespace literals

constexpr std::size_t fnv_prime() noexcept {
	static_assert(sizeof(std::size_t) * CHAR_BIT == 32 || sizeof(std::size_t) * CHAR_BIT == 64);

	if constexpr (sizeof(std::size_t) * CHAR_BIT == 32) {
		return 16777619_usize;
	} else {
		return 1099511628211_usize;
	}
}

constexpr std::size_t fnv_offset_basis() noexcept {
	static_assert(sizeof(std::size_t) * CHAR_BIT == 32 || sizeof(std::size_t) * CHAR_BIT == 64);

	if constexpr (sizeof(std::size_t) * CHAR_BIT == 32) {
		return 2166136261_usize;
	} else {
		return 14695981039346656037_usize;
	}
}

template <std::size_t N>
constexpr std::size_t fnv1a(const unsigned char (&data)[N]) noexcept {
	std::size_t hash = fnv_offset_basis();

	for (std::size_t i = 0; i < N; ++i) {
		hash ^= data[i];
		hash *= fnv_prime();
	}

	return hash;
}

constexpr std::size_t fnv1a(const unsigned char *data, std::size_t length) noexcept {
	std::size_t hash = fnv_offset_basis();

	for (std::size_t i = 0; i < length; ++i) {
		hash ^= data[i];
		hash *= fnv_prime();
	}

	return hash;
}

constexpr std::size_t fnv1a_combine(std::size_t seed, std::size_t hash) noexcept {
	return seed ^ (hash + 0x9e3779b9_usize + (seed << 6) + (seed >> 2));
}

template <typename T>
struct FnvHash {
	constexpr std::size_t operator()(const T &value) const noexcept {
		return fnv1a(reinterpret_cast<const unsigned char (&)[sizeof(T)]>(value));
	}
};

template <typename C, typename T, typename A>
struct FnvHash<std::basic_string<C, T, A>> {
	std::size_t operator()(const std::basic_string<C, T, A> &str) const noexcept {
		return fnv1a(reinterpret_cast<const unsigned char*>(str.data()), str.size());
	}
};

template <typename C, typename T>
struct FnvHash<std::basic_string_view<C, T>> {
	std::size_t operator()(std::basic_string_view<C, T> &sv) const noexcept {
		return fnv1a(reinterpret_cast<const unsigned char*>(sv.data()), sv.size());
	}
};

template <>
struct FnvHash<std::type_index> {
	std::size_t operator()(std::type_index idx) const noexcept {
		const std::hash<std::type_index> hasher;

		return FnvHash<std::size_t>{ }(hasher(idx));
	}
};

} // namespace tmap::detail

#endif
