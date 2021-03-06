#include <catch.hpp>

#include <cstdint>

#include <caterpillar/structures/stg_gate.hpp>
#include <caterpillar/synthesis/lhrs.hpp>
#include <caterpillar/synthesis/strategies/best_fit_mapping_strategy.hpp>
#include <caterpillar/verification/circuit_to_logic_network.hpp>
#include <kitty/static_truth_table.hpp>
#include <mockturtle/algorithms/simulation.hpp>
#include <mockturtle/networks/aig.hpp>
#include <tweedledum/io/write_unicode.hpp>
#include <tweedledum/networks/netlist.hpp>

TEST_CASE( "Best-fit mapping strategy for 3-bit sorting network", "[best_fit_mapping_strategy]" )
{
  using namespace caterpillar;
  using namespace caterpillar::detail;
  using namespace mockturtle;
  using namespace tweedledum;

  aig_network sorter;
  const auto a = sorter.create_pi();
  const auto b = sorter.create_pi();
  const auto c = sorter.create_pi();

  const auto w1 = sorter.create_and( a, b );
  const auto w2 = sorter.create_and( c, w1 );
  const auto w3 = sorter.create_and( !a, !b );
  const auto w4 = sorter.create_and( !c, !w1 );
  const auto w5 = sorter.create_and( !w3, !w4 );
  const auto w6 = sorter.create_or( c, !w3 );

  sorter.create_po( w2 );
  sorter.create_po( w5 );
  sorter.create_po( w6 );

  netlist<stg_gate> circ;
  logic_network_synthesis_stats st;
  best_fit_mapping_strategy<aig_network> strategy;
  logic_network_synthesis(circ, sorter, strategy, {}, {}, &st);

  const auto sorter2 = circuit_to_logic_network<aig_network>(circ, st.i_indexes, st.o_indexes);
  CHECK( sorter2 );
  CHECK( simulate<kitty::static_truth_table<3>>( sorter ) == simulate<kitty::static_truth_table<3>>( *sorter2 ) );
}
