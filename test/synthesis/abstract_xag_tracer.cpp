
#include <catch.hpp>
#include <../test_xag.hpp>

using namespace caterpillar;
using namespace caterpillar::test;
using namespace mockturtle;
using namespace tweedledum;


TEST_CASE("trace simple abs xag", "[abstract_trace_test-1]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 1, false));
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 1, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowd, 1, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 1, false) );
  #endif

}
TEST_CASE("trace simple abs xag 2", "[abstract_trace_test-2]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 2, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 2, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowd, 2, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 2, false) );
  #endif


}

TEST_CASE("trace simple abs xag 3", "[abstract_trace_test-3]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 3, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 3, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 3, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 3, false) );
  #endif


}

TEST_CASE("trace simple abs xag 4", "[abstract_trace_test-4]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 4, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 4, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 4, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 4, false) );
  #endif


  
}

TEST_CASE("trace simple abs xag 5", "[abstract_trace_test-5]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 5, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 5, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 5, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 5, false) );
  #endif

}

TEST_CASE("trace simple abs xag 6", "[abstract_trace_test-6]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 6, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 6, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 6, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 6, false) );
  #endif

}

TEST_CASE("trace simple abs xag with codependent xor outputs", "[abstract_trace_test-7]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 7, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 7, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 7, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 7, false) );
  #endif

}

TEST_CASE("trace simple abs xag with reconvergence", "[abstract_trace_test-8]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 8, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 8, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 8, false) );

  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 8, false) );
  #endif


}

TEST_CASE("trace simple abs xag 9", "[abstract_trace_test-9]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 9, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 9, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 9, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 9, false) );
  #endif



}

TEST_CASE("trace simple abs xag 10", "[abstract_trace_test-10]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 10, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 10, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 10, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 10, false) );
  #endif

}

TEST_CASE("trace simple abs xag 11", "[abstract_trace_test-11]")
{
  
  CHECK(test_tracer(xag_method::abs_xag_lowt, 11, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 11, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 11, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 11, false) );
  #endif


}

TEST_CASE("trace simple abs xag 12", "[abstract_trace_test-12]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 12, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 12, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 12, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 12, false) );
  #endif

}

TEST_CASE("trace simple abs xag 13", "[abstract_trace_test-13]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 13, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 13, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 13, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 13, false) );
  #endif

}

TEST_CASE("trace abs xag and opt weights", "[abstract_trace_test-14]")
{
  
  CHECK(test_tracer(xag_method::abs_xag_lowt, 14, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 14, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 14, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 14, false) );
  #endif
}

TEST_CASE("trace abs xag 15", "[abstract_trace_test-15]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 15, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 15, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 15, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 15, false) );
  #endif

}

TEST_CASE("trace abs xag 16", "[abstract_trace_test-16]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 16, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 16, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 16, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 16, false) );
  #endif

}

TEST_CASE("trace abs xag 17", "[abstract_trace_test-17]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 17, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 17, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 17, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 17, false) );
  #endif

}

TEST_CASE("trace min depth synthesis abs XAG-small", "[abstract_trace_test-18]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 18, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 18, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 18, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 18, false) );
  #endif

}

TEST_CASE("trace min depth synthesis abs XAG-small ", "[abstract_trace_test-19]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 19, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 19, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 19, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 19, false) );
  #endif

}


TEST_CASE("trace abs xag with parity buffer", "[abstract_trace_test-20]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 20, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 20, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 20, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 20, false) );
  #endif

}
TEST_CASE("trace abs xag with included cone", "[abstract_trace_test-21]")
{
  CHECK(test_tracer(xag_method::abs_xag_lowt, 21, false) );
  CHECK(test_tracer(xag_method::abs_xag_lowt_fast, 21, false) );

  CHECK(test_tracer(xag_method::abs_xag_lowd, 21, false) );
  #ifdef USE_iGRAPH
  CHECK(test_tracer(xag_method::abs_xag_lowd_noc, 21, false) );
  #endif
}
