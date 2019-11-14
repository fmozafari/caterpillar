

#include <catch.hpp>
#include <kitty/constructors.hpp>
#include <kitty/dynamic_truth_table.hpp>
#include <kitty/esop.hpp>
#include <sstream>
//#include <tweedledum/algorithms/simulation/classical_simulation.hpp>
//#include <tweedledum/algorithms/synthesis/single_target_gates.hpp>
#include <tweedledum/gates/mcmt_gate.hpp>
#include <tweedledum/gates/mcst_gate.hpp>
#include <tweedledum/io/write_projectq.hpp>
//#include <tweedledum/networks/gdg_network.hpp>
#include <tweedledum/networks/gg_network.hpp>
#include <tweedledum/networks/netlist.hpp>
#include <caterpillar/synthesis/stg_to_mcx.hpp>
#include <caterpillar/structures/stg_gate.hpp>

using namespace caterpillar;
using namespace tweedledum;

template<class Network>
inline std::pair<Network, std::vector<qubit_id>> circuit_and_map(uint32_t qubits)
{
	Network circ;
	for (auto i = 0u; i < qubits; ++i) {
		circ.add_qubit();
	}
	std::vector<qubit_id> map(qubits);
	std::iota(map.begin(), map.end(), 0u);
	return std::make_pair(circ, map);
}


TEST_CASE("Synthesize abc + !a!b!c using exact ESOP synthesis without optimization",
          "[single_target_gates]")
{
	kitty::dynamic_truth_table tt(3);
	kitty::create_from_binary_string(tt, "10000001");
	auto [circ, map] = circuit_and_map<tweedledum::netlist<stg_gate>>(4u);
	stg_from_exact_synthesis()(circ, map, tt );
	

	CHECK(circ.num_gates() == 8u);
	CHECK(circ.num_qubits() == 4u);

}


