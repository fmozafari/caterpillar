
#include <catch.hpp>
#include "../test_xag.hpp"

using namespace caterpillar;
using namespace caterpillar::test;

TEST_CASE("synthesize simple abs xag", "[AXAG synthesis]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 1, false) );
}

TEST_CASE("synthesize simple abs xag 2", "[AXAG synthesis-2]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 2, false) );
}

TEST_CASE("synthesize simple abs xag 3", "[AXAG synthesis-3]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 3, false) );
}

TEST_CASE("synthesize simple abs xag 4", "[AXAG synthesis-4]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 4, false) );
}

TEST_CASE("synthesize simple abs xag 5", "[AXAG synthesis-5]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 5, false) );
}

TEST_CASE("synthesize simple abs xag 6", "[AXAG synthesis-6]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 6, true) );
}

TEST_CASE("synthesize simple abs xag with codependent xor outputs", "[AXAG synthesis-7]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 7, false) );
}

TEST_CASE("synthesize simple abs xag with reconvergence", "[AXAG synthesis-8]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 8, false) );
}

TEST_CASE("synthesize simple abs xag 9", "[AXAG synthesis-9]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 9, false) );
}

TEST_CASE("synthesize simple abs xag 10", "[AXAG synthesis-10]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 10, false) );
}

TEST_CASE("synthesize simple abs xag using pebbling", "[AXAG synthesis-11]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 11, false) );
}

TEST_CASE("pebble simple abs xag 10", "[AXAG synthesis-12]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 12, false) );
}

TEST_CASE("pebble simple abs xag 11", "[AXAG synthesis-13]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 13, false) );
}

TEST_CASE("pebbling abs xag with weights", "[AXAG synthesis-14]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 14, false) );
}

TEST_CASE("min depth synthesis abs XAG", "[AXAG synthesis-15]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 15, false) );
}

TEST_CASE("min depth synthesis abs XAG-2", "[AXAG synthesis-16]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 16, false) );
}

TEST_CASE("min depth synthesis abs xag no copies", "[AXAG synthesis-17]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 17, false) );
}

TEST_CASE("min depth synthesis abs XAG-small", "[AXAG synthesis-18]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 18, false) );
}

TEST_CASE("min depth synthesis abs XAG-small ", "[AXAG synthesis-19]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 19, false) );
}

TEST_CASE("abs xag with parity buffer", "[AXAG synthesis-20]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 20, false) );
}

TEST_CASE("abs xag with included cone", "[AXAG synthesis-21]")
{
  CHECK(xag_synthesis(xag_method::abs_xag_lowt, 21, false) );
}
