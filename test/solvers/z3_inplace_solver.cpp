#include <catch.hpp>

#include <caterpillar/structures/pebbling_view.hpp>
#include <caterpillar/solvers/z3_solver_inplace.hpp>
#include <caterpillar/synthesis/strategies/mapping_strategy.hpp>
#include <caterpillar/synthesis/strategies/pebbling_mapping_strategy.hpp>

#include <mockturtle/networks/xag.hpp>

#include <kitty/dynamic_truth_table.hpp>


using namespace caterpillar;

TEST_CASE(" pebble inplace simple graph ", "[pebin]")
{
	

	mockturtle::xag_network net;

	auto p1 = net.create_pi();
	auto p2 = net.create_pi();
	auto p3 = net.create_pi();

	auto n1 = net.create_xor(p1, p2);
	auto n2 = net.create_and(n1, p3);

	net.create_po(n2);

	pebbling_view<mockturtle::xag_network> pnet ( net );

  auto z3_solver = z3_pebble_inplace_solver( pnet, 2 );

	z3_solver.init();

	do{
		z3_solver.add_step();
	}while (z3_solver.solve() == unsat);

	CHECK(z3_solver.solve() == sat);
	z3_solver.print();
	
	auto strategy = z3_solver.extract_result(true);

  assert( strategy.size() == 3 );
}


TEST_CASE("pebble inplace simple graph 2", "[pebin2]")
{

	mockturtle::xag_network net;

	auto p1 = net.create_pi();
	auto p2 = net.create_pi();
	auto p3 = net.create_pi();

	auto n1 = net.create_xor(p1, p2);
	auto n2 = net.create_xor(n1, p3);


	net.create_po(n2);

	pebbling_view<mockturtle::xag_network> pnet ( net );

  auto z3_solver = z3_pebble_inplace_solver( pnet, 2 );

	z3_solver.init();
	do{
		z3_solver.add_step();
	}while (z3_solver.solve() == unsat);

	CHECK(z3_solver.solve() == sat);
	z3_solver.print();
	
	auto strategy = z3_solver.extract_result(true);
}