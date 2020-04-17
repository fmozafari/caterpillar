/*-------------------------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*------------------------------------------------------------------------------------------------*/
#include <tweedledum/gates/gate_set.hpp>
#include <mockturtle/networks/klut.hpp>
#include <mockturtle/traits.hpp>
#include <caterpillar/structures/abstract_network.hpp>

#pragma once

namespace caterpillar::detail
{
  template<class Ntk>
  static inline std::vector<typename Ntk::node> get_outputs(Ntk const& ntk)
  {
    static_assert(mockturtle::has_foreach_po_v<Ntk>, "Ntk does not implement the foreach_po method");

    std::vector<typename Ntk::node> drivers;
    ntk.foreach_po( [&]( auto const& f ) { drivers.push_back( ntk.get_node( f ) ); } );
    return drivers;
  }

  template<class Ntk>
  static uint32_t resp_num_pis(Ntk const& net)
	{
		if constexpr (std::is_same_v<Ntk, mockturtle::klut_network> || std::is_same_v<Ntk, abstract_network>)
			return net.num_pis() + 2;
		else 
			return net.num_pis() + 1;
	}

  template<class QuantumCircuit>
  static inline int t_cost( QuantumCircuit const& netlist)
  {
    int count = 0;
    netlist.foreach_cgate([&](const auto gate)
    {
      if (gate.gate.is(tweedledum::gate_set::t))
      {
        count++;
      }
    });

    return count;
  }

  static inline int t_cost( const int tof_controls, const int lines )
  {
    switch ( tof_controls )
    {
    case 0u:
    case 1u:
      return 0;

    case 2u:
      return 7;

    case 3u:
      return 16;
      
    default:
      if ( lines - tof_controls - 1 >= ( tof_controls - 1 ) / 2 )
      {
        return 8 * ( tof_controls - 1 );
      }
      else
      {
        return 16 * ( tof_controls - 1 );
      }
    }
  }

  template<class TofNetwork>
  static inline int count_t_gates ( TofNetwork const& netlist ) 
  {
      auto T_number = 0u;
      netlist.foreach_cgate( [&]( const auto& gate ) {
        T_number += t_cost( gate.gate.num_controls(), netlist.size() );
      } );
      return T_number;
  }
  
} // namespace caterpillar::detail