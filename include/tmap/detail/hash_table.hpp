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

#ifndef TMAP_DETAIL_HASH_TABLE_HPP
#define TMAP_DETAIL_HASH_TABLE_HPP

#include <cassert>
#include <climits>
#include <cstddef>
#include <cstdint>

#include <algorithm>
#include <functional>
#include <initializer_list>
#include <memory>
#include <type_traits>
#include <utility>

namespace tmap::detail {

template <typename T, std::enable_if_t<std::is_integral_v<T> && std::is_unsigned_v<T>, int> = 0>
constexpr bool is_power_of_2_or_zero(T x) noexcept {
    return (x & (x - 1)) == 0;
}

template <typename T, std::enable_if_t<std::is_integral_v<T> && std::is_unsigned_v<T>, int> = 0>
constexpr T next_highest_power_of_2(T x) noexcept {
    constexpr auto MAX = static_cast<int>(sizeof(T) * CHAR_BIT / 2);

	--x;

	// should unroll with optimizations; tested with clang 7 and gcc 8
    for (int i = 1; i <= MAX; i *= 2) {
        x |= x >> i;
    }

	++x;
    x += (x == 0);

	return x;
}

template <typename T>
struct RemoveCvrefType {
	using type = std::remove_cv_t<std::remove_reference_t<T>>;
};

template <typename T>
using RemoveCvref = typename RemoveCvrefType<T>::type;

template <typename ...>
struct HeadType { };

template <typename T, typename ...Ts>
struct HeadType<T, Ts...> {
	using type = T;
};

template <typename ...Ts>
using Head = typename HeadType<Ts...>::type;

template <typename T, typename ...Ts>
constexpr T&& head(T &&t, Ts&&...) noexcept {
	return std::forward<T>(t);
}

namespace hash_table {

template <typename T, bool IsConst>
class Iterator;

template <typename T>
using Bucket = std::aligned_storage_t<sizeof(T), alignof(T)>;

template <typename T, typename A>
using BucketAlloc = typename std::allocator_traits<A>::template rebind_alloc<Bucket<T>>;

template <typename T, typename A>
using BucketStorage = std::vector<Bucket<T>, BucketAlloc<T, A>>;

using Dib = std::uint16_t;

template <typename A>
using DibAlloc = typename std::allocator_traits<A>::template rebind_alloc<Dib>;

template <typename A>
using DibStorage = std::vector<Dib, DibAlloc<A>>;

static constexpr inline Dib EMPTY = std::numeric_limits<Dib>::max();

template <typename T, typename H = std::hash<T>, typename E = std::equal_to<T>,
		  typename A = std::allocator<T>>
class HashTable {
public:
	static_assert(std::is_nothrow_move_constructible_v<T>);

	using value_type = T;
	using hasher = H;
	using key_equal = E;
	using allocator_type = A;
	using size_type = std::size_t;
	using iterator = Iterator<T, false>;
	using const_iterator = Iterator<T, true>;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;
	using reference = T&;
	using const_reference = const T&;
	using pointer = typename std::allocator_traits<A>::pointer;
	using const_pointer = typename std::allocator_traits<A>::const_pointer;

	HashTable() = default;

	explicit HashTable(size_type num_buckets)
	: HashTable{ num_buckets, H{ } } { }

	HashTable(size_type num_buckets, const H &hash,
			  const E &equal = E{ }, const A &alloc = A{ })
	: buckets_{ alloc }, dibs_{ alloc}, hash_{ hash }, equal_{ equal } {
		reserve(num_buckets);
	}

	HashTable(size_type num_buckets, const A &alloc)
	: HashTable{ num_buckets, H{ }, E{ }, alloc } { }

	HashTable(size_type num_buckets, const H &hash, const A &alloc)
	: HashTable{ num_buckets, hash, E{ }, alloc } { }

	explicit HashTable(const allocator_type &alloc) noexcept
	: buckets_{ alloc }, dibs_{ alloc } { }

	HashTable(const HashTable &other) : HashTable{
		other,
		AllocTraits::select_on_container_copy_construction(other.get_allocator())
	} { }

	HashTable(const HashTable &other, const A &alloc)
	: HashTable{ other, other.capacity(), alloc } { }

	HashTable(const HashTable &other, size_type num_buckets) : HashTable{
		other,
		num_buckets,
		AllocTraits::select_on_container_copy_construction(other.get_allocator())
	} { }

	HashTable(const HashTable &other, size_type num_buckets, const A &alloc)
	: buckets_{ alloc }, dibs_{ alloc }, hash_{ other.hash_ }, equal_{ other.equal_ } {
		const auto actual_num_buckets = std::max({
			next_highest_power_of_2(num_buckets),
			MINIMUM_CAPACITY,
			next_highest_power_of_2(other.size())
		});

		buckets_.resize(actual_num_buckets);
		dibs_.resize(actual_num_buckets, EMPTY);

		for (size_type i = 0; i < other.capacity(); ++i) {
			if (!other.is_empty(i)) {
				do_insert(other.at(i));
			}
		}

		assert(size_ == other.size_);
	}

	HashTable(HashTable &&other) noexcept : HashTable{ } {
		swap(other);
	}

	HashTable(HashTable &&other, const A &alloc) : HashTable{ } {
		if (alloc == other.get_allocator()) {
			swap(other);

			return;
		}

		reserve(other.size_ * 2);

		for (std::size_t i = 0; i < other.capacity(); ++i) {
			if (!other.is_empty(i)) {
				do_insert(std::move(other.at(i)));
			}
		}

		assert(size_ == other.size_);
	}

	HashTable(std::initializer_list<T> list, size_type bucket_count = 0,
			  const H &hash = H{ }, const E &equal = E{ }, const A &alloc = A{ })
	: HashTable{ 0, hash, equal, alloc } {
		reserve(std::max(bucket_count, list.size() * 2));

		for (const T &elem : list) {
			insert(elem);
		}
	}

	~HashTable() {
		clear();
	}

	HashTable& operator=(const HashTable &other) {
		HashTable to_swap{ other };
		swap(to_swap);

		return *this;
	}

	HashTable& operator=(HashTable &&other) noexcept {
		swap(other);

		return *this;
	}

	allocator_type get_allocator() const noexcept {
		return A{ buckets_.get_allocator() };
	}

	bool empty() const noexcept {
		return size_ == 0;
	}

	size_type size() const noexcept {
		return size_;
	}

	size_type capacity() const noexcept {
		assert(buckets_.size() == dibs_.size());
		assert(is_power_of_2_or_zero(buckets_.size()));

		return static_cast<size_type>(buckets_.size());
	}

	iterator begin() noexcept {
		if (empty()) {
			return end();
		}

		return make_iterator(0);
	}

	const_iterator begin() const noexcept {
		return cbegin();
	}

	const_iterator cbegin() const noexcept {
		if (empty()) {
			return cend();
		}

		return make_iterator(0);
	}

	iterator end() noexcept {
		return make_iterator(capacity());
	}

	const_iterator end() const noexcept {
		return cend();
	}

	const_iterator cend() const noexcept {
		return make_iterator(capacity());
	}

	iterator find(const T &value) noexcept {
		const auto found = do_find(value);

		if (found == NOT_FOUND) {
			return end();
		}

		return make_iterator(found);
	}

	const_iterator find(const T &value) const noexcept {
		const auto found = do_find(value);

		if (found == NOT_FOUND) {
			return cend();
		}

		return make_iterator(found);
	}

	std::pair<iterator, bool> insert(const T &value) {
		const auto found = do_find(value);

		if (found != NOT_FOUND) {
			return { make_iterator(found), false };
		}

		return { make_iterator(do_insert(value)), true };
	}

	std::pair<iterator, bool> insert(T &&value) {
		const auto found = do_find(value);

		if (found != NOT_FOUND) {
			return { make_iterator(found), false };
		}

		return { make_iterator(do_insert(std::move(value))), true };
	}

	iterator erase(const_iterator pos) noexcept {
		const auto idx = static_cast<size_type>(pos.bucket_ - pos.first_);
		const auto next_idx = do_erase(idx);

		if (next_idx == NOT_FOUND) {
			return end();
		}

		return make_iterator(next_idx);
	}

	size_type erase(const T &value) noexcept {
		const auto found = do_find(value);

		if (found == NOT_FOUND) {
			return 0;
		}

		do_erase(found);

		return 1;
	}

	double load_factor() const noexcept {
		return static_cast<double>(size_) / static_cast<double>(capacity());
	}

	double max_load_factor() const noexcept {
		return 0.5;
	}

	void reserve(size_type new_capacity) {
		if (new_capacity <= capacity()) {
			return;
		}

		const auto actual_new_capacity =
			std::max(next_highest_power_of_2(new_capacity), MINIMUM_CAPACITY);

		*this = HashTable{ *this, actual_new_capacity };
	}

	void swap(HashTable &other) noexcept {
		using std::swap;

		swap(buckets_, other.buckets_);
		swap(dibs_, other.dibs_);
		swap(size_, other.size_);
		swap(hash_, other.hash_);
		swap(equal_, other.equal_);
	}

	void clear() noexcept {
		for (size_type i = 0; i < capacity(); ++i) {
			if (dibs_[i] != EMPTY) {
				destroy(i);
			}
		}

		size_ = 0;
	}

private:
	using AllocTraits = std::allocator_traits<A>;

	static constexpr inline size_type NOT_FOUND = std::numeric_limits<size_type>::max();
	static constexpr inline size_type MINIMUM_CAPACITY = 16;

	iterator make_iterator(size_type index) noexcept {
		assert(index <= capacity());

		return { buckets_.data() + index, dibs_.data() + index,
				 buckets_.data(), buckets_.data() + capacity() };
	}

	const_iterator make_iterator(size_type index) const noexcept {
		assert(index <= capacity());

		return { buckets_.data() + index, dibs_.data() + index,
				 buckets_.data(), buckets_.data() + capacity() };
	}

	T& at(size_type index) noexcept {
		assert(index < capacity());
		assert(dibs_[index] != EMPTY);

		return reinterpret_cast<T&>(buckets_[index]);
	}

	const T& at(size_type index) const noexcept {
		assert(index < capacity());
		assert(dibs_[index] != EMPTY);

		return reinterpret_cast<const T&>(buckets_[index]);
	}

	T* get(size_type index) noexcept {
		assert(index <= capacity());

		return reinterpret_cast<T*>(buckets_.data() + index);
	}

	const T* get(size_type index) const noexcept {
		assert(index <= capacity());

		return reinterpret_cast<const T*>(buckets_.data() + index);
	}

	size_type do_find(const T &value) const noexcept {
		if (empty()) {
			return NOT_FOUND;
		}

		const auto start_idx = static_cast<size_type>(hash_(value) & (capacity() - 1));

		for (auto i = start_idx; i < capacity(); ++i) {
			if (dibs_[i] == EMPTY) {
				return NOT_FOUND;
			}

			if (equal_(value, at(i))) {
				return i;
			}
		}

		for (size_type i = 0; i < start_idx; ++i) {
			if (dibs_[i] == EMPTY) {
				return NOT_FOUND;
			}

			if (equal_(value, at(i))) {
				return i;
			}
		}

		return NOT_FOUND;
	}

	size_type do_insert(const T &value) {
		return do_emplace(hash_(value), value);
	}

	size_type do_insert(T &&value) {
		return do_emplace(hash_(value), std::move(value));
	}

	template <typename ...Ts, std::enable_if_t<std::is_constructible_v<T, Ts&&...>, int> = 0>
	size_type do_emplace(std::size_t hash, Ts &&...ts) {
		reserve((size_ + 1) * 2);

		++size_;
		const auto bucket = static_cast<size_type>(hash & (capacity() - 1));

		if (dibs_[bucket] == EMPTY) {
			construct(bucket, 0, std::forward<Ts>(ts)...);
			assert(hash_(at(bucket)) == hash);

			return bucket;
		}

		const auto [swap_bucket, dib] = find_swap_bucket_after(bucket);
		const auto init_swap_dib = dibs_[swap_bucket];
		const auto [empty_bucket, swapped_dib] = find_empty_bucket_after(swap_bucket);

		construct(empty_bucket, swapped_dib, std::move(at(swap_bucket)));
		destroy(empty_bucket);

		try {
			construct(swap_bucket, dib, std::forward<Ts>(ts)...);
		} catch (...) {
			construct(swap_bucket, init_swap_dib, std::move(at(empty_bucket)));
			destroy(empty_bucket);
			--size_;

			throw;
		}

		assert(hash_(at(swap_bucket)) == hash);

		return swap_bucket;
	}

	std::pair<size_type, Dib> find_swap_bucket_after(size_type idx) const noexcept {
		assert(dibs_[idx] != EMPTY);

		Dib dib = 1;

		for (auto i = idx + 1; i < capacity(); ++i, ++dib) {
			if (dibs_[i] < dib) {
				return { i, dib };
			}
		}

		for (size_type i = 0; i < idx; ++i, ++dib) {
			if (dibs_[i] < dib) {
				return { i, dib };
			}
		}

		__builtin_unreachable();
	}

	std::pair<size_type, Dib> find_empty_bucket_after(size_type idx) const noexcept {
		assert(dibs_[idx] != EMPTY);
		assert(size_ < capacity());

		Dib dib = dibs_[idx] + 1;

		for (size_type i = idx + 1; i < capacity(); ++i, ++dib) {
			if (dibs_[i] == EMPTY) {
				return { i, dib };
			}
		}

		for (size_type i = 0; i < idx; ++i, ++dib) {
			if (dibs_[i] == EMPTY) {
				return { i, dib };
			}
		}

		__builtin_unreachable();
	}

	size_type do_erase(size_type bucket) noexcept {
		assert(bucket < capacity());
		assert(dibs_[bucket] != EMPTY);

		destroy(bucket);
		shift_left_until_empty(bucket);

		for (size_type i = bucket; i < capacity(); ++i) {
			if (dibs_[i] != EMPTY) {
				return i;
			}
		}

		return NOT_FOUND;
	}

	void shift_left_until_empty(size_type bucket) noexcept {
		assert(!empty());
		assert(bucket < capacity());
		assert(dibs_[bucket] == EMPTY);

		--size_;

		const auto next_bucket = [this, bucket]() -> size_type {
			if (bucket + 1 == capacity()) {
				return 0;
			}

			return bucket + 1;
		}();

		if (!should_move_from(next_bucket)) {
			return;
		}

		const auto move_from = [this](size_type src, size_type dst) {
			assert(src < capacity());
			assert(dst < capacity());
			assert(dibs_[dst] == EMPTY);
			assert(dibs_[src] != EMPTY && dibs_[src] > 0);

			construct(dst, dibs_[src] - 1, std::move(at(src)));
			destroy(src);
			dibs_[src] = EMPTY;
		};

		move_from(next_bucket, bucket);

		if (next_bucket != 0) {
			for (auto i = next_bucket + 1; i < capacity(); ++i) {
				if (!should_move_from(i)) {
					return;
				}

				move_from(i, i - 1);
			}

			if (!should_move_from(0)) {
				return;
			}

			move_from(0, capacity() - 1);
		}

		for (size_type i = 1; i < bucket; ++i) {
			if (!should_move_from(i)) {
				return;
			}

			move_from(i, i - 1);
		}
	}

	bool should_move_from(size_type index) const noexcept {
		return dibs_[index] != EMPTY && dibs_[index] > 0;
	}

	template <typename ...Ts, std::enable_if_t<std::is_constructible_v<T, Ts&&...>, int> = 0>
	void construct(size_type bucket, Dib dib, Ts &&...ts)
	noexcept(std::is_nothrow_constructible_v<T, Ts&&...>) {
		assert(bucket < capacity());
		assert(dibs_[bucket] == EMPTY);
		assert(dib != EMPTY);

		auto alloc{ get_allocator() };
		AllocTraits::construct(alloc, get(bucket), std::forward<Ts>(ts)...);
		dibs_[bucket] = dib;
	}

	void destroy(size_type bucket) noexcept {
		assert(bucket < capacity());
		assert(dibs_[bucket] != EMPTY);

		auto alloc{ get_allocator() };
		AllocTraits::destroy(alloc, get(bucket));
		dibs_[bucket] = EMPTY;
	}

	bool is_empty(size_type bucket) const noexcept {
		assert(bucket < capacity());

		return dibs_[bucket] == EMPTY;
	}

	BucketStorage<T, A> buckets_;
	DibStorage<A> dibs_{ buckets_.get_allocator() };
	size_type size_ = 0;
	H hash_;
	E equal_;
};

template <typename T, bool IsConst>
class Iterator {
public:
	template <typename, typename, typename, typename>
	friend class HashTable;

	template <typename, bool>
	friend class Iterator;

	using difference_type = std::ptrdiff_t;
	using iterator_category = std::bidirectional_iterator_tag;
	using pointer = std::conditional_t<IsConst, const T*, T*>;
	using reference = std::conditional_t<IsConst, const T&, T&>;
	using value_type = T;

	template <bool B = IsConst, std::enable_if_t<B, int> = 0>
	Iterator(Iterator<T, !B> other) noexcept
	: bucket_{ other.bucket_ }, dib_{ other.dib_ },
	  first_{ other.first_ }, last_{ other.last_ } { }

	reference operator*() const {
		assert(bucket_ >= first_ && bucket_ < last_);

		return reinterpret_cast<reference>(*bucket_);
	}

	pointer operator->() const {
		assert(bucket_ >= first_ && bucket_ < last_);

		return reinterpret_cast<pointer>(bucket_);
	}

	Iterator& operator++() {
		assert(bucket_ >= first_ && bucket_ < last_);

		if (bucket_ == last_) {
			return *this;
		}

		for (++bucket_, ++dib_; bucket_ < last_ && *dib_ == EMPTY; ++bucket_, ++dib_) { }

		return *this;
	}

	Iterator operator++(int) {
		const auto ret{ *this };

		++*this;

		return ret;
	}

	Iterator& operator--() {
		assert(bucket_ > first_ && bucket_ <= last_);

		if (bucket_ == first_) {
			return *this;
		}

		for (--bucket_, --dib_; bucket_ > first_ && *dib_ == EMPTY; --bucket_, --dib_) { }

		return *this;
	}

	Iterator operator--(int) {
		const auto ret{ *this };

		--*this;

		return ret;
	}

	friend bool operator==(const Iterator &lhs, const Iterator &rhs) noexcept {
		assert(lhs.first_ == rhs.first_);
		assert(lhs.last_ == rhs.last_);

		return lhs.bucket_ == rhs.bucket_;
	}

	friend bool operator!=(const Iterator &lhs, const Iterator &rhs) noexcept {
		assert(lhs.first_ == rhs.first_);
		assert(lhs.last_ == rhs.last_);

		return !(lhs == rhs);
	}

private:
	using Bucket = std::conditional_t<IsConst, const Bucket<T>, Bucket<T>>;

	Iterator(Bucket *current, const Dib *current_dib, Bucket *first, Bucket *last) noexcept
	: bucket_{ current }, dib_{ current_dib }, first_{ first }, last_{ last } {
		assert(first <= last);

		for (; bucket_ < last_ && *dib_ == EMPTY; ++bucket_, ++dib_) { }
	}

	Bucket *bucket_;
	const Dib *dib_;
	Bucket *first_;
	Bucket *last_;
};

} // namespace hash_table

template <typename T, typename H = std::hash<T>, typename E = std::equal_to<T>,
		  typename A = std::allocator<T>>
using HashTable = hash_table::HashTable<T, H, E, A>;

} // namespace tmap::detail

#endif
