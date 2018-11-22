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

#include <tmap/detail/hash_table.hpp>

#include <algorithm>
#include <vector>

#include <catch2/catch.hpp>

TEST_CASE("HashTable basics") {
	tmap::detail::HashTable<int> table;

	REQUIRE(table.empty());
	REQUIRE(table.size() == 0);

	const auto [iter, inserted] = table.insert(5);

	REQUIRE(iter != table.end());
	REQUIRE(*iter == 5);
	REQUIRE(inserted);
	REQUIRE_FALSE(table.empty());
	REQUIRE(table.size() == 1);

	const auto found = table.find(5);

	REQUIRE(found != table.end());
	REQUIRE(*found == 5);

	const auto [again_iter, inserted_again] = table.insert(5);

	REQUIRE_FALSE(inserted_again);
	REQUIRE(again_iter == iter);
	REQUIRE(again_iter != table.end());
	REQUIRE(*again_iter == 5);

	REQUIRE(table.erase(again_iter) == table.end());
	REQUIRE(table.find(5) == table.end());
	REQUIRE(table.size() == 0);
	REQUIRE(table.empty());
}

TEST_CASE("HashTable multi basics") {
	tmap::detail::HashTable<int> table;

	REQUIRE(table.empty());
	REQUIRE(table.size() == 0);

	REQUIRE(table.insert(5).second);
	REQUIRE(table.insert(10).second);
	REQUIRE(table.insert(15).second);
	REQUIRE(table.insert(20).second);
	REQUIRE(table.insert(25).second);
	REQUIRE(table.insert(30).second);

	REQUIRE(table.size() == 6);
	REQUIRE_FALSE(table.empty());

	std::vector<int> actual{ table.cbegin(), table.cend() };
	std::sort(actual.begin(), actual.end());
	static const std::vector<int> EXPECTED{ 5, 10, 15, 20, 25, 30 };
	REQUIRE(actual == EXPECTED);

	REQUIRE(table.erase(5) == 1);
	REQUIRE(table.erase(5) == 0);
	REQUIRE(table.erase(10) == 1);
	REQUIRE(table.erase(10) == 0);
	REQUIRE(table.erase(15) == 1);
	REQUIRE(table.erase(15) == 0);
	REQUIRE(table.erase(20) == 1);
	REQUIRE(table.erase(20) == 0);
	REQUIRE(table.erase(25) == 1);
	REQUIRE(table.erase(25) == 0);
	REQUIRE(table.erase(30) == 1);
	REQUIRE(table.erase(30) == 0);

	actual.assign(table.cbegin(), table.cend());
	std::sort(actual.begin(), actual.end());
	REQUIRE(actual.empty());

	REQUIRE(table.size() == 0);
	REQUIRE(table.empty());
}
