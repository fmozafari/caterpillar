
#include <catch.hpp>
#include "test_xag.hpp"

using namespace caterpillar;
using namespace caterpillar::test;

TEST_CASE("synthesize simple xag", "[XAG synthesis]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 1, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 1, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 1, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 1, false) == true);
  #endif
}

TEST_CASE("synthesize simple xag 2", "[XAG synthesis-2]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 2, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 2, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 2, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 2, false) == true);
  #endif
}

TEST_CASE("synthesize simple xag 3", "[XAG synthesis-3]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 3, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 3, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 3, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 3, false) == true);
  #endif

}

TEST_CASE("synthesize simple xag 4", "[XAG synthesis-4]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 4, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 4, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 4, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 4, false) == true);
  #endif

  
}

TEST_CASE("synthesize simple xag 5", "[XAG synthesis-5]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 5, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 5, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 5, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 5, false) == true);
  #endif

}

TEST_CASE("synthesize simple xag 6", "[XAG synthesis-6]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 6, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 6, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 6, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 6, false) == true);
  #endif

}

TEST_CASE("synthesize simple xag with codependent xor outputs", "[XAG synthesis-7]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 7, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 7, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 7, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 7, false) == true);
  #endif
}

TEST_CASE("synthesize simple xag with reconvergence", "[XAG synthesis-8]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 8, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 8, false) == true);

  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 8, false) == true);
  #endif

  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 8, false) == true);
  #endif

}

TEST_CASE("synthesize simple xag 9", "[XAG synthesis-9]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 9, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 9, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 9, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 9, false) == true);
  #endif


}

TEST_CASE("synthesize simple xag 10", "[XAG synthesis-10]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 10, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 10, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 10, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 10, false) == true);
  #endif

}

TEST_CASE("synthesize simple xag using pebbling", "[XAG synthesis-11]")
{
  
  CHECK(xag_synthesis(xag_method::xag_lowt, 11, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 11, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 11, false) == true);
  #endif
  #ifdef USE_Z3
  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=2;
  CHECK(xag_synthesis(xag_method::xag_pebb, 11, false) == true);
  #endif

}

TEST_CASE("pebble simple xag 10", "[XAG synthesis-12]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 12, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 12, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 12, false) == true);
  #endif
  #ifdef USE_Z3
  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=4;
  CHECK(xag_synthesis(xag_method::xag_pebb, 12, false) == true);
  #endif

}

TEST_CASE("pebble simple xag 11", "[XAG synthesis-13]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 13, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 13, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 13, false) == true);
  #endif
  #ifdef USE_Z3
  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=28;
  CHECK(xag_synthesis(xag_method::xag_pebb, 13, false, peb_ps) == true);
  #endif
}

TEST_CASE("pebbling XAG with weights", "[XAG synthesis-14]")
{
  
  CHECK(xag_synthesis(xag_method::xag_lowt, 14, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 14, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 14, false) == true);
  #endif
  #ifdef USE_Z3
  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=4;
  peb_ps.conflict_limit = 1000000;
  peb_ps.optimize_weight = true;
  peb_ps.verbose = false;
  CHECK(xag_synthesis(xag_method::xag_pebb, 14, false, peb_ps) == true);
  #endif
}

TEST_CASE("min depth synthesis XAG", "[XAG synthesis-15]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 15, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 15, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 15, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 15, false) == true);
  #endif
}

TEST_CASE("min depth synthesis XAG-2", "[XAG synthesis-16]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 16, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 16, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 16, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 16, false) == true);
  #endif
}

TEST_CASE("min depth synthesis XAG no copies", "[XAG synthesis-17]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 17, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 17, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 17, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 17, false) == true);
  #endif
}

TEST_CASE("min depth synthesis XAG-small", "[XAG synthesis-18]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 18, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 18, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 18, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 18, false) == true);
  #endif
}

TEST_CASE("min depth synthesis XAG-small ", "[XAG synthesis-19]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 19, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 19, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 19, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 19, false) == true);
  #endif
}


TEST_CASE("min depth synthesis parity buffer", "[XAG synthesis-20]")
{
  CHECK(xag_synthesis(xag_method::xag_lowt, 20, false) == true);
  CHECK(xag_synthesis(xag_method::xag_lowd, 20, false) == true);
  #ifdef USE_iGRAPH
  CHECK(xag_synthesis(xag_method::xag_dfit, 20, false) == true);
  #endif
  #ifdef USE_Z3
  CHECK(xag_synthesis(xag_method::xag_pebb, 20, false) == true);
  #endif
}