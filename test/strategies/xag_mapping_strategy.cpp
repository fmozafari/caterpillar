
#include <catch.hpp>

#include <caterpillar/synthesis/lhrs.hpp>
#include <caterpillar/synthesis/strategies/xag_mapping_strategy.hpp>
#include <caterpillar/verification/circuit_to_logic_network.hpp>
#include <mockturtle/networks/xag.hpp>
#include <tweedledum/networks/netlist.hpp>
#include <tweedledum/io/write_unicode.hpp>
#include <tweedledum/io/write_projectq.hpp>

#include <mockturtle/io/write_verilog.hpp>
#include <mockturtle/io/write_dot.hpp>

#include <caterpillar/structures/stg_gate.hpp>
#include <mockturtle/algorithms/simulation.hpp>
#include <kitty/static_truth_table.hpp>
#include <fmt/format.h>


TEST_CASE("synthesize simple xag", "[XAG synthesis]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto a = xag.create_pi();
  auto b = xag.create_pi();
  auto c = xag.create_pi();
  auto d = xag.create_and( a, b );
  auto e = xag.create_xor( d, c );
  xag.create_po( e );

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  
  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag )[0];
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk )[0];

  CHECK(tt_xag == tt_ntk);
  
}


TEST_CASE("synthesize simple xag 2", "[XAG synthesis-2]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto a = xag.create_pi();
  auto b = xag.create_pi();
  auto c = xag.create_pi();
  auto d = xag.create_pi();
  auto e = xag.create_xor(a, b);
  auto f = xag.create_xor(e, c);
  auto g = xag.create_and(a, f);
  auto h = xag.create_and(g, d);
  auto i = xag.create_xor(h, d);
  xag.create_po( i );

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );
  

  CHECK(tt_xag == tt_ntk);
  
}
TEST_CASE("synthesize simple xag 3", "[XAG synthesis-3]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto a = xag.create_pi();
  auto b = xag.create_pi();
  auto c = xag.create_pi();
  auto d = xag.create_pi();
  auto e = xag.create_xor(a, b);
  auto f = xag.create_xor(e, c);
  auto g = xag.create_xor(a, f);
  auto h = xag.create_and(g, d);
  xag.create_po( h );

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag )[0];
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk )[0];

  CHECK(tt_xag == tt_ntk);

  
}

TEST_CASE("synthesize simple xag 4", "[XAG synthesis-4]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto s = xag.create_pi();
  auto t5 = xag.create_pi();
  auto t2 = xag.create_pi();
  auto t15 = xag.create_pi();
  auto t12 = xag.create_pi();
  auto t8 = xag.create_pi();
  auto t7 = xag.create_pi();
  auto t13 = xag.create_pi();

  auto t6 = xag.create_xor(t5, t2);
  auto t16 = xag.create_xor(t15, t12);
  auto t18 = xag.create_xor(t6, t16);
  auto t9 = xag.create_xor(t7, t8);
  auto t14 = xag.create_xor(t13, t12);
  auto t19 = xag.create_xor(t9, t14);
  auto t22 = xag.create_xor(t18, t19);
  auto s0 = xag.create_and(t22, s);


  xag.create_po( s0 );

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag )[0];
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk )[0];

  CHECK(tt_xag == tt_ntk);

  
}

TEST_CASE("synthesize simple xag 5", "[XAG synthesis-5]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x0 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();
  auto n10 = xag.create_xor(x6, x0);
  auto n9 = xag.create_xor(x5, x3);
  auto n16 = xag.create_xor(n10, n9);
  auto n20 = xag.create_xor(n16, x4);
  auto n21 = xag.create_xor(n20, x5);
  auto n32 = xag.create_and(n16, n21);
  xag.create_po(n32);

  netlist<stg_gate> qnet;
  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  
  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag )[0];
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk )[0];
  CHECK(tt_xag == tt_ntk);

}

TEST_CASE("synthesize simple xag 6", "[XAG synthesis-6]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x0 = xag.create_pi();
  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();

  auto n10 = xag.create_xor(x6, x0);
  auto n9 = xag.create_xor(x5, x3);
  auto n16 = xag.create_xor(n10, n9);
  auto n20 = xag.create_xor(n16, x4);
  auto n22 = xag.create_xor(n20, x1);
  auto n11 = xag.create_xor(x3, x0);
  auto n25 = xag.create_xor(n22, n11);
  auto n13 = xag.create_xor(x2, x1);
  auto n29 = xag.create_xor(n25, n13);
  auto n37 = xag.create_xor(n10, n29);
  xag.create_po(n37);
  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<8>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<8>>( *ntk );

  CHECK(tt_xag == tt_ntk);

}

TEST_CASE("synthesize simple xag with codependent xor outputs", "[XAG synthesis-7]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x0 = xag.create_pi();
  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();
  auto x7 = xag.create_pi();
  auto x8 = xag.create_pi();

  auto z0 = xag.create_xor(x1 , x2 );
  auto z1 = xag.create_xor(x3 , x4 );
  auto z2 = xag.create_xor(z0 , z1 );
  auto z3 = xag.create_xor(x0 , z2 );
  auto z4 = xag.create_xor(x6 , z1 );
  auto z5 = xag.create_xor(x5 , z4 );
  auto z6 = xag.create_xor(z5 , z3 );
  auto z7 = xag.create_xor(x7 , x1 );
  auto z8 = xag.create_xor(x8 , x3 );
  auto z9 = xag.create_xor(z7 , z8 );
  auto z10 = xag.create_xor(z9 , z3 );
  
  xag.create_po(z3);
  xag.create_po(!z6);
  xag.create_po(z10);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<9>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<9>>( *ntk );

  CHECK(tt_xag == tt_ntk);

}

TEST_CASE("synthesize simple xag with reconvergence", "[XAG synthesis-8]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x2 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();
  auto x7 = xag.create_pi();
  auto x4 = xag.create_pi();


  auto n12 = xag.create_xor(x7, x5);
  auto n13 = xag.create_xor(n12, x6);
  auto n17 = xag.create_xor(n12, x2);
  auto n19 = xag.create_xor(n17, n13);
  auto n20 = xag.create_xor(x4, x6);
  auto n22 = xag.create_and(n19, n20);
  xag.create_po(n22);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<8>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<8>>( *ntk );

  CHECK(tt_xag == tt_ntk);

}

TEST_CASE("synthesize simple xag 8", "[XAG synthesis-9]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x0 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();
  auto x7 = xag.create_pi();

  auto n19 = xag.create_xor(x7, x0);
  auto n20 = xag.create_and(n19, x6);
  auto n21 = xag.create_and(n19, n20);
  auto n26 = xag.create_xor(x5, x0);
  auto n27 = xag.create_xor(n26, n20);
  auto n29 = xag.create_and(x7, n27);
  auto n32 = xag.create_and(n21, n29);

  xag.create_po(n32);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );

  CHECK(tt_xag == tt_ntk);


}

TEST_CASE("synthesize simple xag 9", "[XAG synthesis-10]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x6 = xag.create_pi();

  auto n13 = xag.create_xor(x2, x1);
  auto n14 = xag.create_xor(n13, x6);
  auto n18 = xag.create_xor(n14, x6);
  auto n38 = xag.create_and(n14, n18);
  
  xag.create_po(n38);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );

  CHECK(tt_xag == tt_ntk);


}

#ifdef USE_Z3
TEST_CASE("synthesize simple xag using pebbling", "[XAG pebbling synthesis]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();

  auto n1 = xag.create_and(x1, x2);
  auto n2 = xag.create_xor(x2, x3);
  auto n3 = xag.create_xor(n1, n2);
  auto n4 = xag.create_and(n2, n3);
  
  xag.create_po(n4);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=2;

  xag_pebbling_mapping_strategy strategy(peb_ps);
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<3>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<3>>( *ntk );

  CHECK(tt_xag == tt_ntk);

}

TEST_CASE("pebble simple xag 10", "[XAG synthesis-10]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;
  auto xag = xag_network();

  auto x0 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();
  auto x7 = xag.create_pi();

  auto n19 = xag.create_xor(x7, x0);
  auto n20 = xag.create_and(n19, x6);
  auto n21 = xag.create_and(n19, n20);
  auto n26 = xag.create_xor(x5, x0);
  auto n27 = xag.create_xor(n26, n20);
  auto n29 = xag.create_and(x7, n27);
  auto n32 = xag.create_xor(n21, n29);

  xag.create_po(n32);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=4;
  xag_pebbling_mapping_strategy strategy (peb_ps);
    
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );

  CHECK(tt_xag == tt_ntk);


}

TEST_CASE("pebble simple xag 11", "[XAG synthesis-11]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  auto xag = xag_network();

  auto z0 = xag.create_pi();
  auto z1 = xag.create_pi();
  auto z3 = xag.create_pi();
  auto z4 = xag.create_pi();
  auto z6 = xag.create_pi();
  auto z7 = xag.create_pi();
  auto z9 = xag.create_pi();
  auto z10 = xag.create_pi();
  auto z15 = xag.create_pi();
  auto z16 = xag.create_pi();

  auto t46 = xag.create_xor(z15 , z16);
  auto t49 = xag.create_xor(z9 , z10);
  auto t53 = xag.create_xor(z0 , z3);
  auto t54 = xag.create_xor(z6 , z7);
  auto t58 = xag.create_xor(z4 , t46);
  auto t59 = xag.create_xor(z3 , t54);
  auto t63 = xag.create_xor(t49 , t58);
  auto t64 = xag.create_xor(z4 , t59);
  auto t66 = xag.create_xor(z1 , t63);
  auto s3 = xag.create_xor(t53 , t66);
  auto s1pre = xag.create_xor(t64 , s3);
  auto s1 = xag.create_not(s1pre);

  xag.create_po(s3);
  xag.create_po(s1);


  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  pebbling_mapping_strategy_params peb_ps;
  peb_ps.pebble_limit=28;
  xag_pebbling_mapping_strategy strategy (peb_ps);
    
  logic_network_synthesis( qnet, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<10>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<10>>( *ntk );

  CHECK(tt_xag == tt_ntk);

}

TEST_CASE("pebbling XAG with weights", "[minweight]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;

  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();

  auto n1 = xag.create_xor(x1, x2);
  auto n2 = xag.create_and(x2, n1);
  auto n3 = xag.create_and(x2, n2);
  auto n4 = xag.create_xor(x3, x4);
  auto n5 = xag.create_xor(x5, x6);
  auto n6 = xag.create_and(n4, n5);
  auto n7 = xag.create_xor(n6, n4);
  auto n8 = xag.create_and(x3, n7);
  auto n9 = xag.create_and(n8, n3);

  xag.create_po(n9);

  pebbling_mapping_strategy_params pss;
  pss.pebble_limit = 4;
  pss.conflict_limit = 100000;
  pss.optimize_weight = true;
  pss.verbose = false;
  xag_pebbling_mapping_strategy strategy (pss);
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;
  ps.verbose = false;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );
  auto tt_xag = simulate<kitty::static_truth_table<6>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<6>>( *ntk );

  CHECK(tt_xag == tt_ntk);
}
#endif

TEST_CASE("min depth synthesis XAG", "[mindepth]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;

  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();

  auto n1 = xag.create_xor(x1, x2);
  auto n2 = xag.create_xor(x3, x4);
  auto n3 = xag.create_and(n1, n2);
  auto n4 = xag.create_xor(x4, x5);
  auto n5 = xag.create_and(n4, x6);
  auto n6 = xag.create_and(n3, n5);

  xag.create_po(n6);

  xag_low_depth_mapping_strategy strategy;
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;
  ps.verbose = true;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );
  write_unicode(rev);
  auto tt_xag = simulate<kitty::static_truth_table<6>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<6>>( *ntk );
  CHECK(tt_xag == tt_ntk);
}


TEST_CASE("min depth synthesis XAG-2", "[mindepth]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;

  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();


  auto n1 = xag.create_and(x1, x2);
  auto n2 = xag.create_and(x2, x3);
  auto n3 = xag.create_and(!x1, x2);
  auto n4 = xag.create_xor(n1, n2);
  auto n5 = xag.create_xor(n4, n3);

  xag.create_po(n5);

  xag_low_depth_mapping_strategy strategy;
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;
  ps.verbose = false;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<3>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<3>>( *ntk );

  CHECK(tt_xag == tt_ntk);
}

TEST_CASE("min depth synthesis XAG no copies", "[mindepth3]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;
  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();


  auto n1 = xag.create_xor(x1, x2);
  auto n2 = xag.create_and(n1, x3);
  auto n3 = xag.create_and(x3, x4);
  auto n4 = xag.create_and(x4, x5);
  auto n5 = xag.create_and( n2, n3);

  xag.create_po(n5);
  xag.create_po(n4);
  xag.create_po(n1);


  xag_depth_fit_mapping_strategy strategy;
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;
  ps.verbose = false;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );
  auto tt_xag = simulate<kitty::static_truth_table<5>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<5>>( *ntk );

  CHECK(tt_xag == tt_ntk);
}

TEST_CASE("min depth synthesis XAG-small", "[mindepth4]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;
  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();



  auto n1 = xag.create_xor(x1, x2);
  auto n2 = xag.create_xor(x2, x3);
  auto n3 = xag.create_and(n1, n2);
  auto n4 = xag.create_and(n2, x4);
  auto n5 = xag.create_and( n4, n3);

  xag.create_po(n5);


  xag_low_depth_mapping_strategy strategy;
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );
  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );

  CHECK(tt_xag == tt_ntk);
}

TEST_CASE("min depth synthesis XAG-small ", "[mindepth5]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;
  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();



  auto n1 = xag.create_and(x1, x2);
  auto n2 = xag.create_and(x1, x3);
  auto n3 = xag.create_and(x1, x4);
  auto n4 = xag.create_and(n2, n3);
  auto n5 = xag.create_and( n4, n1);

  xag.create_po(n5);


  xag_low_depth_mapping_strategy strategy;
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;
  ps.verbose=false;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );
  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );
  CHECK(tt_xag == tt_ntk);
}


TEST_CASE("min depth synthesis parity buffer", "[mindepth6]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  xag_network xag;
  auto x1 = xag.create_pi();
  auto x2 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();

  auto n1 = xag.create_xor(x1, x2);
  auto n2 = xag.create_xor(n1, x3);
  auto n3 = xag.create_xor(n2, x4);
  auto n4 = xag.create_xor(n3, x5);

  auto n5 = xag.create_xor(n3, n4);
  auto n6 = xag.create_and(n5, x3);

  xag.create_po(n6);


  xag_low_depth_mapping_strategy strategy;
  netlist<stg_gate> rev;

  logic_network_synthesis_stats st;
  logic_network_synthesis_params ps;
  ps.verbose=false;

  logic_network_synthesis(rev, xag, strategy, {}, ps, &st );

  auto tt_xag = simulate<kitty::static_truth_table<4>>( xag );
  const auto ntk = circuit_to_logic_network<xag_network, netlist<stg_gate>>( rev, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );
  CHECK(tt_xag == tt_ntk);
}


