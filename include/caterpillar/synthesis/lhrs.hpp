/*------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Mathias Soeken
| Author(s): Giulia Meuli
*-----------------------------------------------------------------------------*/
#pragma once
#include "../structures/stg_gate.hpp"
#include "strategies/mapping_strategy.hpp"

#include <array>
#include <cstdint>
#include <fmt/format.h>
#include <mockturtle/algorithms/cut_enumeration/spectr_cut.hpp>
#include <mockturtle/traits.hpp>
#include <mockturtle/utils/node_map.hpp>
#include <mockturtle/utils/stopwatch.hpp>
#include <mockturtle/views/topo_view.hpp>
#include <tweedledum/algorithms/synthesis/stg.hpp>
#include <stack>
#include <fmt/format.h>
#include <variant>
#include <vector>

using Qubit = tweedledum::qubit_id;
using SetQubits = std::vector<Qubit>;

namespace caterpillar
{

namespace mt = mockturtle;

struct logic_network_synthesis_params
{
  /*! \brief Be verbose. */
  bool verbose{false};
};

struct logic_network_synthesis_stats
{
  /*! \brief Total runtime. */
  mockturtle::stopwatch<>::duration time_total{0};

  /*! \brief Required number of ancilla. */
  uint32_t required_ancillae{0u};

  /*! \brief output qubits. */
  std::vector<uint32_t> o_indexes;

  /*! \brief input qubits. */
  std::vector<uint32_t> i_indexes;

  void report() const
  {
    std::cout << fmt::format( "[i] total time = {:>5.2f} secs\n", mockturtle::to_seconds( time_total ) );
  }
};

namespace detail
{

template<class QuantumNetwork, class LogicNetwork, class SingleTargetGateSynthesisFn>
class logic_network_synthesis_impl
{
  using node_t = typename LogicNetwork::node;
public:
  logic_network_synthesis_impl( QuantumNetwork& qnet, LogicNetwork const& ntk,
                                mapping_strategy<LogicNetwork>& strategy,
                                SingleTargetGateSynthesisFn const& stg_fn,
                                logic_network_synthesis_params const& ps,
                                logic_network_synthesis_stats& st )
      : qnet( qnet ), ntk( ntk ), strategy( strategy ), stg_fn( stg_fn ), ps( ps ), st( st )
  {
  }

  bool run()
  {
    mockturtle::stopwatch t( st.time_total );
    prepare_inputs();
    prepare_constant( false );
    if ( ntk.get_node( ntk.get_constant( false ) ) != ntk.get_node( ntk.get_constant( true ) ) )
      prepare_constant( true );

    if ( const auto result = strategy.compute_steps( ntk ); !result )
    {
    
      std::cout << "[i] strategy could not be computed\n";
      return false;
    }
    strategy.foreach_step( [&]( auto node, auto action ) {
      std::visit(
          overloaded{
              []( auto ) {},
              [&]( compute_copy_action const& action ) {
                const auto t = request_ancilla();
                
                if ( ps.verbose )
                {
                  fmt::print("[i] copies for node {} to qubit {}\n", ntk.node_to_index( node ), t);
                }
   
                copy_node( action.copies, t );
                copies[node].push(t);
              },
              [&]( uncompute_copy_action const& action ) {
                const auto t = copies[node].front();
                
                if ( ps.verbose )
                {
                  fmt::print("[i] remove copies for {} from qubit {}\n",  node , t);
                }
                copy_node( action.copies, t );
                copies[node].pop();
                // this would allow reusing qubits but increases the T depth
                //release_ancilla( t );
              },
              [&]( compute_oncopies_action const& action ) {
                const auto t = copies[node].front();

                if (ps.verbose) 
                {
                  fmt::print("[i] compute {} on its copies on qubit {}\n", node, t); 
                  fmt::print("with leaves {}\n", fmt::join(action.leaves, " "));
                  fmt::print("and copies {}\n", fmt::join(action.copies, " "));

                }
                std::vector<uint32_t> rem_leaves;
                std::set_symmetric_difference( 
                  action.leaves.begin(), action.leaves.end(), action.copies.begin(), action.copies.end(), 
                  std::back_inserter(rem_leaves) );

                compute_big_xor( t, rem_leaves);
                node_to_qubit[node].push( t );
              },
              [&]( uncompute_oncopies_action const& action ) {
                const auto t = node_to_qubit[node].top();

                if (ps.verbose) 
                {
                  fmt::print("[i] uncompute {} on its copies from qubit {}\n", node, t); 
                  fmt::print("with leaves {}\n", fmt::join(action.leaves, " "));
                  fmt::print("and copies {}\n", fmt::join(action.copies, " "));
                }

                std::vector<uint32_t> rem_leaves;
                std::set_symmetric_difference( 
                  action.leaves.begin(), action.leaves.end(), action.copies.begin(), action.copies.end(), 
                  std::back_inserter(rem_leaves) );

                compute_big_xor( t, rem_leaves);
                node_to_qubit[node].pop();
              },
              [&]( compute_action const& action ) {
                const auto t = request_ancilla();
                node_to_qubit[node].push(t);
                if ( ps.verbose )
                {
                  fmt::print("[i] compute {} in qubit {}\n", node, t);
                  if ( action.leaves )   
                    fmt::print(" with leaves: {} \n", fmt::join(*action.leaves, " ")); 
                }
                if ( action.cell_override )
                {
                  const auto [func, leaves] = *action.cell_override;
                  compute_node_as_cell( node, t, func, leaves );
                }
                else if (action.leaves)
                {
                  compute_big_xor( t, *action.leaves );
                }
                else
                {
                  compute_node( node, t ); 
                }
              },
              [&]( uncompute_action const& action ) {
                const auto t = node_to_qubit[node].top();
                if ( ps.verbose )
                {  
                  fmt::print("[i] uncompute {} from qubit {}\n", node, t);
                  if ( action.leaves )   
                    fmt::print(" with leaves: {} \n", fmt::join(*action.leaves, " ")); 
                }
                if ( action.cell_override )
                {
                  const auto [func, leaves] = *action.cell_override;
                  compute_node_as_cell( node, t, func, leaves );
                }
                else if (action.leaves)
                {
                  compute_big_xor(  t, *action.leaves );
                }
                else
                {
                  compute_node( node, t );
                }
                release_ancilla( t );
              },
              [&]( compute_inplace_action const& action ) {
                const auto t = node_to_qubit[ntk.index_to_node( action.target_index )].top() ;
                node_to_qubit[node].push(t);

                if ( ps.verbose )
                {  
                  fmt::print("[i] compute {} inplace onto node {}\n", node , action.target_index);
                  if ( action.leaves )   
                    fmt::print(" with leaves: {} \n", fmt::join(*action.leaves, " ")); 
                }
                if (action.leaves)
                {
                  compute_big_xor(t, *action.leaves);
                }
                else
                {  
                  compute_node_inplace( node, t );
                }
              },
              [&]( uncompute_inplace_action const& action ) {
                const auto t = node_to_qubit[node].top();
                if ( ps.verbose )
                {
                  fmt::print("[i] uncompute {} inplace to {}\n", node , action.target_index);
                  if ( action.leaves )   
                    fmt::print(" with leaves: {} \n", fmt::join(*action.leaves, " ")); 
                }
                if (action.leaves)
                {
                  compute_big_xor(  t, *action.leaves );
                }
                else 
                {  
                  compute_node_inplace( node, t );
                }
              },
              [&] (compute_level_action const& action){
                compute_level(action.node_to_cones);
              },
              [&] (uncompute_level_action const& action){
                uncompute_level(action.node_to_cones);
              }},
          action );
    } );

    prepare_outputs();

    return true;
  }

private:
  void prepare_inputs()
  {
    /* prepare primary inputs of logic network */
    ntk.foreach_pi( [&]( auto n ) {
      std::stack<uint32_t> sta;
      sta.push( qnet.num_qubits() );
      node_to_qubit[n] = sta;
      st.i_indexes.push_back( node_to_qubit[n].top() );
      qnet.add_qubit();
    } );
    ntk.foreach_gate( [&]( auto n ) {
      std::stack<uint32_t> sta;
      node_to_qubit[n] = sta;
    } );
  
  }

  void prepare_constant( bool value )
  {
    const auto f = ntk.get_constant( value );
    const auto n = ntk.get_node( f );
    if ( ntk.fanout_size( n ) == 0 )
      return;
    const auto v = ntk.constant_value( n ) ^ ntk.is_complemented( f );
    node_to_qubit[n].push( qnet.num_qubits() );
    qnet.add_qubit();
    if ( v )
      qnet.add_gate( tweedledum::gate::pauli_x, node_to_qubit[n].top() );
  }

  uint32_t request_ancilla()
  {
    if ( free_ancillae.empty() )
    {
      const auto r = qnet.num_qubits();
      st.required_ancillae++;
      qnet.add_qubit();
      return r;
    }
    else
    {
      const auto r = free_ancillae.top();
      free_ancillae.pop();
      return r;
    }
  }

  void prepare_outputs()
  {
    std::unordered_map<mt::node<LogicNetwork>, mt::signal<LogicNetwork>> node_to_signals;
    ntk.foreach_po( [&]( auto s ) {
      auto node = ntk.get_node( s );

      if ( const auto it = node_to_signals.find( node ); it != node_to_signals.end() ) //node previously referred
      {
        auto new_i = request_ancilla();

        qnet.add_gate( tweedledum::gate::cx, node_to_qubit[ntk.node_to_index( node )].top(), new_i );
        if ( ntk.is_complemented( s ) != ntk.is_complemented( node_to_signals[node] ) )
        {
          qnet.add_gate( tweedledum::gate::pauli_x, new_i );
        }
        st.o_indexes.push_back( new_i );
      }
      else //node never referred
      {
        if ( ntk.is_complemented( s ) )
        {
          qnet.add_gate( tweedledum::gate::pauli_x, node_to_qubit[ntk.node_to_index( node )].top() );
        }
        node_to_signals[node] = s;
        st.o_indexes.push_back( node_to_qubit[ntk.node_to_index( node )].top() );
      }
    } );
  }

  void release_ancilla( uint32_t q )
  {
    free_ancillae.push( q );
  }

  template<int Fanin>
  std::array<uint32_t, Fanin> get_fanin_as_literals( mt::node<LogicNetwork> const& n )
  {
    std::array<uint32_t, Fanin> controls;
    ntk.foreach_fanin( n, [&]( auto const& f, auto i ) {
      controls[i] = ( ntk.node_to_index( ntk.get_node( f ) ) << 1 ) | ntk.is_complemented( f );
    } );
    return controls;
  }

  SetQubits get_fanin_as_qubits( mt::node<LogicNetwork> const& n )
  {
    SetQubits controls;
    ntk.foreach_fanin( n, [&]( auto const& f ) {
      assert( !ntk.is_complemented( f ) );
      controls.push_back( tweedledum::qubit_id( node_to_qubit[ntk.node_to_index( ntk.get_node( f ) )].top() ) );
    } );
    return controls;
  }

  void compute_big_xor( uint32_t const t, std::vector<uint32_t> const leaves)
  {
    for ( auto control : leaves )
    {
      auto c =  node_to_qubit[control].top();
      if (c != t)
      {     
        qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c ), tweedledum::qubit_id( t ) );
      }
    }
  }

  void copy_node( std::vector<uint32_t> copies, uint32_t t )
  {
    for( auto cp : copies)
    {
      assert(node_to_qubit[cp].top() != t);
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( node_to_qubit[cp].top() ), tweedledum::qubit_id( t ) );
    }
  }

  void compute_node( mt::node<LogicNetwork> const& node, uint32_t t)
  {
    if constexpr ( mt::has_is_and_v<LogicNetwork> )
    {
      if ( ntk.is_and( node ) )
      {
        auto controls = get_fanin_as_literals<2>( node );
        auto node0 = ntk.index_to_node( controls[0] >> 1 );
        auto node1 = ntk.index_to_node( controls[1] >> 1 );

        SetQubits pol_controls;     
        pol_controls.emplace_back( node_to_qubit[node0].top(), controls[0] & 1 );
        pol_controls.emplace_back( node_to_qubit[node1].top(), controls[1] & 1 );
        
        compute_and( pol_controls, t );
        return;
      }
    }
    if constexpr ( mt::has_is_or_v<LogicNetwork> )
    {
      if ( ntk.is_or( node ) )
      {
        auto controls = get_fanin_as_literals<2>( node );

        SetQubits pol_controls;
        pol_controls.emplace_back( node_to_qubit[ntk.index_to_node( controls[0] >> 1 )].top(), !( controls[0] & 1 ) );
        pol_controls.emplace_back( node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(), !( controls[1] & 1 ) );

        compute_or( pol_controls, t );
        return;
      }
    }
    if constexpr ( mt::has_is_xor_v<LogicNetwork> )
    {
      if ( ntk.is_xor( node ) )
      {
        
        auto controls = get_fanin_as_literals<2>( node );
        compute_xor( node_to_qubit[ntk.index_to_node( controls[0] >> 1 )].top(),
                     node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
                     ( controls[0] & 1 ) != ( controls[1] & 1 ), t );
        return;
      }
    }
    if constexpr ( mt::has_is_xor3_v<LogicNetwork> )
    {
      if ( ntk.is_xor3( node ) )
      {
        auto controls = get_fanin_as_literals<3>( node );

        /* Is XOR3 in fact an XOR2? */
        if ( ntk.is_constant( ntk.index_to_node( controls[0] >> 1 ) ) )
        {
          compute_xor( node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
                       node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(),
                       ( ( controls[0] & 1 ) != ( controls[1] & 1 ) ) != ( controls[2] & 1 ),
                       t );
        }
        else
        {
          compute_xor3(
              node_to_qubit[ntk.index_to_node( controls[0] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(),
              ( ( controls[0] & 1 ) != ( controls[1] & 1 ) ) != ( controls[2] & 1 ),
              t );
        }
        return;
      }
    }
    if constexpr ( mt::has_is_maj_v<LogicNetwork> )
    {
      if ( ntk.is_maj( node ) )
      {
        auto controls = get_fanin_as_literals<3>( node );
        /* Is XOR3 in fact an AND or OR? */
        if ( ntk.is_constant( ntk.index_to_node( controls[0] >> 1 ) ) )
        {
          if ( controls[0] & 1 )
          {
            SetQubits pol_controls;
            pol_controls.emplace_back( node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(), !( controls[1] & 1 ) );
            pol_controls.emplace_back( node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(), !( controls[2] & 1 ) );

            compute_or( pol_controls, tweedledum::qubit_id( t ) );
          }
          else
          {
            SetQubits pol_controls;
            pol_controls.emplace_back( node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(), controls[1] & 1 );
            pol_controls.emplace_back( node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(), controls[2] & 1 );

            compute_and( pol_controls, tweedledum::qubit_id( t ) );
          }
        }
        else
        {
          compute_maj(
              node_to_qubit[ntk.index_to_node( controls[0] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(),
              controls[0] & 1, controls[1] & 1, controls[2] & 1, t );
        }
        return;
      }
    }
    if constexpr ( mt::has_node_function_v<LogicNetwork> )
    {
      kitty::dynamic_truth_table tt = ntk.node_function( node );
      auto clone = tt.construct();
      kitty::create_parity( clone );

      if ( tt == clone )
      {
        const auto controls = get_fanin_as_qubits( node );
        compute_xor_block( controls, tweedledum::qubit_id( t ) );
      }
      else
      {
        // In this case, the procedure works a bit different and retrieves the
        // controls directly as mapped qubits.  We assume that the inputs cannot
        // be complemented, e.g., in the case of k-LUT networks.
        const auto controls = get_fanin_as_qubits( node );
        compute_lut( ntk.node_function( node ), controls, tweedledum::qubit_id( t ) );
      }
    }
  }

  void compute_node_as_cell( mt::node<LogicNetwork> const& node, uint32_t t, kitty::dynamic_truth_table const& func, std::vector<uint32_t> const& leave_indexes )
  {
    (void)node;

    /* get control qubits */
    SetQubits controls;
    for ( auto l : leave_indexes )
    {
      controls.push_back( tweedledum::qubit_id( node_to_qubit[ntk.node_to_index( l )].top() ) );
    }

    compute_lut( func, controls, tweedledum::qubit_id( t ) );
  }

  void compute_node_inplace( mt::node<LogicNetwork> const& node, uint32_t t )
  {
    if constexpr ( mt::has_is_xor_v<LogicNetwork> )
    {
      if ( ntk.is_xor( node ) )
      {
        auto controls = get_fanin_as_literals<2>( node );
        compute_xor_inplace( node_to_qubit[ntk.index_to_node( controls[0] >> 1 )].top(),
                             node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
                             ( controls[0] & 1 ) != ( controls[1] & 1 ), t );
        return;
      }
    }
    if constexpr ( mt::has_is_xor3_v<LogicNetwork> )
    {
      if ( ntk.is_xor3( node ) )
      {
        auto controls = get_fanin_as_literals<3>( node );

        /* Is XOR3 in fact an XOR2? */
        if ( ntk.is_constant( ntk.index_to_node( controls[0] >> 1 ) ) )
        {
          compute_xor_inplace(
              node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(),
              ( ( controls[0] & 1 ) != ( controls[1] & 1 ) ) != ( controls[2] & 1 ),
              t );
        }
        else
        {
          compute_xor3_inplace(
              node_to_qubit[ntk.index_to_node( controls[0] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[1] >> 1 )].top(),
              node_to_qubit[ntk.index_to_node( controls[2] >> 1 )].top(),
              ( ( controls[0] & 1 ) != ( controls[1] & 1 ) ) != ( controls[2] & 1 ),
              t );
        }
        return;
      }
    }
    if constexpr ( mt::has_node_function_v<LogicNetwork> )
    {
      const auto controls = get_fanin_as_qubits( node );
      compute_xor_block( controls, tweedledum::qubit_id( t ) );
    }
  }

  void compute_and( SetQubits controls, uint32_t t )
  {
    qnet.add_gate( tweedledum::gate::mcx, controls, SetQubits{{t}} );
  }

  void compute_or( SetQubits controls, uint32_t t )
  {
    qnet.add_gate( tweedledum::gate::mcx, controls, SetQubits{{t}} );
    qnet.add_gate( tweedledum::gate::pauli_x, tweedledum::qubit_id( t ) );
  }

  void compute_xor( uint32_t c1, uint32_t c2, bool inv, uint32_t t)
  {
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), tweedledum::qubit_id( t ) );
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), tweedledum::qubit_id( t ) );
    if ( inv )
      qnet.add_gate( tweedledum::gate::pauli_x, tweedledum::qubit_id( t ) );
  }

  void compute_xor3( uint32_t c1, uint32_t c2, uint32_t c3, bool inv, uint32_t t )
  {
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), tweedledum::qubit_id( t ) );
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), tweedledum::qubit_id( t ) );
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c3 ), tweedledum::qubit_id( t ) );
    if ( inv )
      qnet.add_gate( tweedledum::gate::pauli_x, tweedledum::qubit_id( t ) );
  }

  void compute_maj( uint32_t c1, uint32_t c2, uint32_t c3, bool p1, bool p2, bool p3, uint32_t t )
  {
    if ( p1 )
      qnet.add_gate( tweedledum::gate::pauli_x, c1 );
    if ( !p2 ) /* control 2 behaves opposite */
      qnet.add_gate( tweedledum::gate::pauli_x, c2 );
    if ( p3 )
      qnet.add_gate( tweedledum::gate::pauli_x, c3 );

    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), c2 );
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c3 ), c1 );
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c3 ), t );

    SetQubits controls;
    controls.push_back( tweedledum::qubit_id( c1 ) );
    controls.push_back( tweedledum::qubit_id( c2 ) );
    qnet.add_gate( tweedledum::gate::mcx, controls, SetQubits{{t}} );

    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c3 ), c1 );
    qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), c2 );

    if ( p3 )
      qnet.add_gate( tweedledum::gate::pauli_x, c3 );
    if ( !p2 )
      qnet.add_gate( tweedledum::gate::pauli_x, c2 );
    if ( p1 )
      qnet.add_gate( tweedledum::gate::pauli_x, c1 );
  }

  void compute_xor_block( SetQubits const& controls, Qubit t )
  {
    for ( auto c : controls )
    {
      if ( c != t )
        qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c ), t );
    }
  }

  void compute_lut( kitty::dynamic_truth_table const& function,
                    SetQubits const& controls, Qubit t )
  {
    auto qubit_map = controls;
    qubit_map.push_back( t );
    stg_fn( qnet, qubit_map, function );
  }

  void compute_xor_inplace( uint32_t c1, uint32_t c2, bool inv, uint32_t t )
  {

    if ( c1 == t && c2 != t)
    {
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), c1 );
    }
    else if ( c2 == t && c1 !=t)
    {
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), c2 );
    }
    else if (c1 != t && c2!= t && c1 != c2)
    {
      //std::cerr << "[e] target does not match any control in in-place\n";
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), t );
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), t );
    }
    if ( inv )
      qnet.add_gate( tweedledum::gate::pauli_x, t );
  }

  void compute_xor3_inplace( uint32_t c1, uint32_t c2, uint32_t c3, bool inv, uint32_t t )
  {
    if ( c1 == t )
    {
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), tweedledum::qubit_id( c1 ) );
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c3 ), tweedledum::qubit_id( c1 ) );
    }
    else if ( c2 == t )
    {
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), c2 );
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c3 ), c2 );
    }
    else if ( c3 == t )
    {
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), c3 );
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), c3 );
    }
    else
    {
      //std::cerr << "[e] target does not match any control in in-place\n";
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c1 ), t );
      qnet.add_gate( tweedledum::gate::cx, tweedledum::qubit_id( c2 ), t );
    }
    if ( inv )
      qnet.add_gate( tweedledum::gate::pauli_x, t );
  }

  void compute_level(std::vector<std::pair<uint32_t, std::vector<action_sets>>> const& node_to_cones)
  {
    //for each node in a level
      //insert copies on a requested ancillae
    std::map<node_t, std::vector<uint32_t>> id_to_tcp;
    for(auto node : node_to_cones)
    {      
      std::vector<uint32_t> roots_targets (2);
      for(auto i = 0; i < 2 ; i++)
      {
        auto cone = node.second[i];
        if(cone.copies.empty())
        {
          if(cone.leaves.size() == 1)
          {
            roots_targets[i] = node_to_qubit[cone.node];
          }
          else
          {
            roots_targets[i] = cone.target.empty() ? request_ancilla() : node_to_qubit[cone.target[0]].top();
          }
          
        }
        else
        {
          for (auto c : cone.copies)
          {
            auto tcp = request_ancilla();
            auto qc = node_to_qubit[c].top();
            qnet.add_gate(tweedledum::gate::cx, tweedledum::qubit_id( qc ), tcp );
            roots_targets[i] = tcp;
          }
        }
      }
      id_to_tcp[node.first] = roots_targets;
    }

    for(auto node : node_to_cones)
    {
      auto id = node.first;
      
      for(auto i = 0; i < 2 ; i++)
      {
        auto cone = node.second[i];
        std::vector<node_t> rem_leaves;
        std::set_symmetric_difference( 
                  cone.leaves.begin(), cone.leaves.end(), cone.copies.begin(), cone.copies.end(), 
                  std::back_inserter(rem_leaves) );
        
        auto root = cone.node;
        auto tcp = id_to_tcp[id][i];
        for( auto l : rem_leaves )
        {
          auto lq = node_to_qubit[l].top();
          if((lq != tcp) && (l != root))
          {
            qnet.add_gate(tweedledum::gate::cx, tweedledum::qubit_id( lq ), tcp );
          }
        }
        node_to_qubit[root].push(tcp);

      }

      auto target = request_ancilla();
      compute_node(id, target );
      node_to_qubit[id].push(target);

      for(auto i = 0; i < 2 ; i++)
      {
        auto cone = node.second[i];
        std::vector<uint32_t> rem_leaves;
        std::set_symmetric_difference( 
                  cone.leaves.begin(), cone.leaves.end(), cone.copies.begin(), cone.copies.end(), 
                  std::back_inserter(rem_leaves) );
        
        auto root = cone.node;
        auto tcp = id_to_tcp[id][i];
        for( auto l : rem_leaves )
        {
          auto lq = node_to_qubit[l].top();
          if((lq != tcp) && (l != root))
          {
            qnet.add_gate(tweedledum::gate::cx, tweedledum::qubit_id( lq ), tcp );    
          }
        }
        node_to_qubit[root].pop();
      }
    }
    //remove copies
    for(auto node : node_to_cones)
    {      
      for(auto i = 0; i < 2 ; i++)
      {
        auto cone = node.second[i];
        if(!cone.copies.empty())
        {
          for (auto c : cone.copies)
          {
            auto tcp = id_to_tcp[node.first][i];
            auto qc = node_to_qubit[c].top();
            qnet.add_gate(tweedledum::gate::cx, tweedledum::qubit_id( qc ), tcp );
          }
        }
      }
    }

  }

  void uncompute_level(std::vector<std::pair<uint32_t, std::vector<action_sets>>> const& node_to_cones)
  {
    //for each node in a level
      //insert copies on a requested ancillae

    for(auto node : node_to_cones)
    {
      auto id = node.first;
      
      for(auto i = 0; i < 2 ; i++)
      {
        auto cone = node.second[i];
        auto root = cone.node;
        auto t = cone.target.empty() ? request_ancilla() : cone.target[0];
        for( auto l : cone.leaves )
        {
          auto lq = node_to_qubit[l].top();
          if((lq != t) && (l != root))
            qnet.add_gate(tweedledum::gate::cx, tweedledum::qubit_id( lq ), t );
          node_to_qubit[root].push(t);
        }
      }

      auto target = node_to_qubit[id].top();
      compute_node(id, target );
      node_to_qubit[id].pop();

      for(auto i = 0; i < 2 ; i++)
      {
        auto cone = node.second[i];
        auto root = cone.node;
        auto t = node_to_qubit[root].top();
        for( auto l : cone.leaves )
        {
          auto lq = node_to_qubit[l].top();
          if(lq != t)
            qnet.add_gate(tweedledum::gate::cx, tweedledum::qubit_id( lq ), t );
          node_to_qubit[root].pop();
        }
      }
    }

  }


private:
  QuantumNetwork& qnet;
  LogicNetwork const& ntk;
  mapping_strategy<LogicNetwork>& strategy;
  SingleTargetGateSynthesisFn const& stg_fn;
  logic_network_synthesis_params const& ps;
  logic_network_synthesis_stats& st;
  std::unordered_map<uint32_t, std::stack<uint32_t>> node_to_qubit;
  std::stack<uint32_t> free_ancillae;
  /* stores for each root of the cone a queue of qubits where its copies are and its previous location */
  std::unordered_map<uint32_t, std::queue<uint32_t>> copies;
}; // namespace detail

} // namespace detail

/*! \brief Hierarchical synthesis based on a logic network
 *
 * This algorithm used hierarchical synthesis and computes a reversible network
 * for each gate in the circuit and computes the intermediate result to an
 * ancilla line.  The node may be computed out-of-place or in-place.  The
 * order in which nodes are computed and uncomputed, and whether they are
 * computed out-of-place or in-place is determined by a separate mapper
 * component `MappingStrategy` that is passed as template parameter to the
 * function.
 */
template<class QuantumNetwork, class LogicNetwork,
         class SingleTargetGateSynthesisFn = tweedledum::stg_from_pprm>
bool logic_network_synthesis( QuantumNetwork& qnet, LogicNetwork const& ntk,
                              mapping_strategy<LogicNetwork>& strategy,
                              SingleTargetGateSynthesisFn const& stg_fn = {},
                              logic_network_synthesis_params const& ps = {},
                              logic_network_synthesis_stats* pst = nullptr )
{
  static_assert( mt::is_network_type_v<LogicNetwork>, "LogicNetwork is not a network type" );

  logic_network_synthesis_stats st;
  detail::logic_network_synthesis_impl<QuantumNetwork, LogicNetwork, SingleTargetGateSynthesisFn> impl( qnet,
                                                                                                        ntk,
                                                                                                        strategy,
                                                                                                        stg_fn,
                                                                                                        ps, st );
  const auto result = impl.run();
  if ( ps.verbose )
  {
    st.report();
  }

  if ( pst )
  {
    *pst = st;
  }

  return result;
}

} /* namespace caterpillar */
