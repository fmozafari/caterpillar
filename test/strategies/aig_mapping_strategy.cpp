#include <catch.hpp>
#include <mockturtle/networks/aig.hpp>
#include <mockturtle/algorithms/simulation.hpp>

#include <kitty/kitty.hpp>

#include <caterpillar/synthesis/lhrs.hpp>
#include <caterpillar/synthesis/strategies/aig_mapping_strategy.hpp>
#include <caterpillar/verification/circuit_to_logic_network.hpp>

#include <tweedledum/io/write_unicode.hpp>

using namespace caterpillar;
using namespace mockturtle;
using namespace tweedledum;

TEST_CASE("aig_mapping" , "[map small aig with min depth")
{
  aig_network aig;

  auto a = aig.create_pi();
  auto b = aig.create_pi();
  auto c = aig.create_pi();
  auto d = aig.create_pi();

  auto n1 = aig.create_and(a, b);
  auto n2 = aig.create_and(b, c);
  auto n3 = aig.create_and(c, d);
  auto n4 = aig.create_and(n1, n2);
  auto n5 = aig.create_and(n3, n4);

  aig.create_po(n5);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  aig_mapping_strategy_stats stm;
  aig_mapping_strategy strategy (&stm);

  logic_network_synthesis( qnet, aig, strategy, {}, ps, &st );


  auto tt_aig = simulate<kitty::static_truth_table<4>>( aig );
  const auto ntk = circuit_to_logic_network<aig_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );

  CHECK (tt_aig == tt_ntk);
  CHECK (stm.t_count == 4 * 5);
  CHECK (stm.t_depth == 4);
  CHECK (stm.qubits == 9);


}

TEST_CASE("aig_mapping2" , "[map small aig with min depth-2]")
{
  aig_network aig;

  auto a = aig.create_pi();
  auto b = aig.create_pi();
  auto c = aig.create_pi();
  auto d = aig.create_pi();
  auto e = aig.create_pi();
  auto f = aig.create_pi();

  auto n1 = aig.create_and(a, b);
  auto n2 = aig.create_and(d, c);
  auto n3 = aig.create_and(e, d);
  auto n4 = aig.create_and(n1, n2);
  auto n5 = aig.create_and(n3, f);
  auto n6 = aig.create_and(n4, n5);
  auto n7 = aig.create_not(n6);


  aig.create_po(n7);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  aig_mapping_strategy_stats stm;
  aig_mapping_strategy strategy (&stm);

  logic_network_synthesis( qnet, aig, strategy, {}, ps, &st );

  auto tt_aig = simulate<kitty::static_truth_table<4>>( aig );
  const auto ntk = circuit_to_logic_network<aig_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<4>>( *ntk );

  CHECK(tt_aig == tt_ntk);
  CHECK (stm.t_count == 4 * 6);
  CHECK (stm.t_depth == 4);
  CHECK (stm.qubits == 12);


}

TEST_CASE("synthesize aig enabling extra ancillae", "[enabling_ancillae]")
{
  aig_network aig;

  auto a = aig.create_pi();
  auto b = aig.create_pi();
  auto c = aig.create_pi();

  auto n1 = aig.create_and(a, b);
  auto n2 = aig.create_and(b, c);
  auto n3 = aig.create_and(n1, n2);

  aig.create_po(n3);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  aig_mapping_strategy strategy ({}, true);

  logic_network_synthesis( qnet, aig, strategy, {}, ps, &st );

  auto tt_aig = simulate<kitty::static_truth_table<3>>( aig );
  const auto ntk = circuit_to_logic_network<aig_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<3>>( *ntk );

  CHECK(tt_aig == tt_ntk);

}

TEST_CASE("synthesize aig enabling extra ancillae, multiple conflicts", "[enabling_ancillae_mult_confl]")
{
  aig_network aig;

  auto a = aig.create_pi();
  auto b = aig.create_pi();

  auto n1 = aig.create_and(!a, b);
  auto n2 = aig.create_and(a, !b);
  auto n3 = aig.create_and(n1, n2);

  aig.create_po(n3);

  netlist<stg_gate> qnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = true;

  aig_mapping_strategy strategy ({}, true);

  logic_network_synthesis( qnet, aig, strategy, {}, ps, &st );

  auto tt_aig = simulate<kitty::static_truth_table<3>>( aig );
  const auto ntk = circuit_to_logic_network<aig_network, netlist<stg_gate>>( qnet, st.i_indexes, st.o_indexes );
  auto tt_ntk = simulate<kitty::static_truth_table<3>>( *ntk );

  write_unicode(qnet);
  CHECK(tt_aig == tt_ntk);

}