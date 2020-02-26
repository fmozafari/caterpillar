/*-------------------------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*------------------------------------------------------------------------------------------------*/

#pragma once

#include "mapping_strategy.hpp"
#include <caterpillar/structures/stg_gate.hpp>
#include <caterpillar/structures/pebbling_view.hpp>
#include <caterpillar/synthesis/strategies/pebbling_mapping_strategy.hpp>
#include <caterpillar/solvers/z3_solver.hpp>
#include <mockturtle/networks/xag.hpp>
#include <mockturtle/views/topo_view.hpp>
#include <tweedledum/networks/netlist.hpp>

#include <algorithm>


namespace caterpillar
{

struct action_sets
{
  mockturtle::node<mockturtle::xag_network> node;
  std::vector<uint32_t> target;
  std::vector<uint32_t> leaves;

  action_sets( mockturtle::node<mockturtle::xag_network> node, std::vector<uint32_t> leaves, std::vector<uint32_t> target = {} )
      : node(node), target(target), leaves(leaves) {};
};

inline std::vector<uint32_t> sym_diff(std::vector<uint32_t> first, std::vector<uint32_t> second)
{
  std::vector<uint32_t> diff;
  std::set_symmetric_difference( first.begin(), first.end(), second.begin(), second.end(), std::back_inserter(diff) );
  return diff;
}

inline bool first_cone_included_in_second(std::vector<uint32_t> first, std::vector<uint32_t> second)
{
  /* if second is included in first */
  bool is_included = true;
  for(auto l : first)
  {
    if (std::find( second.begin(), second.end(), l) == second.end())
    {
      is_included = false;
      break;
    }
  }
  return is_included;
}


inline std::vector<std::vector<uint32_t>> compute_fi( mockturtle::node<mockturtle::xag_network> node, mockturtle::xag_network const& xag)
  {
    std::vector<std::vector<uint32_t>> fi (xag.size());

    if ( xag.is_and( node ) || xag.is_pi(node))
    {
      fi[ xag.node_to_index(node) ] = { xag.node_to_index(node) };
    }

    else
    {      
      std::vector<uint32_t> fanin;
      xag.foreach_fanin(node, [&]( auto si ) {
        fanin.push_back(xag.node_to_index(xag.get_node(si)));
      } );

      
      fi[xag.node_to_index( node )] = sym_diff( fi[fanin[0]], fi[fanin[1]] );
    }
    return fi;
  }

inline  std::vector<action_sets> get_fi_target( mockturtle::node<mockturtle::xag_network> node, mockturtle::xag_network const& xag, std::vector<std::vector<uint32_t>>& fi )
  {
    std::vector<action_sets> chs; 
    std::vector<mockturtle::node<mockturtle::xag_network>> fanins;
    bool mono = false;

    xag.foreach_fanin( node, [&]( auto si ) {
      auto fanin = xag.get_node( si );

      auto set = action_sets(fanin, fi[xag.node_to_index( fanin )] ) ; 
      chs.push_back( set ); 
      if(xag.is_pi(fanin) || xag.is_and(fanin) ) mono = true;
      
      fanins.push_back(fanin);
    } );

    assert( chs.size() == 2 ); //contains max two elements

    if ( !mono )
    {
      if ( first_cone_included_in_second(fi[xag.node_to_index(chs[1].node)], fi[xag.node_to_index(chs[0].node)]) )
        std::reverse(chs.begin(), chs.end());

      /* search a target for first */
      /* empty if first is included */
      std::set_difference(fi[xag.node_to_index(chs[0].node)].begin(), fi[xag.node_to_index(chs[0].node)].end(), 
        fi[xag.node_to_index( chs[1].node )].begin(), fi[xag.node_to_index( chs[1].node )].end(), std::inserter(chs[0].target, chs[0].target.begin()));

      /* set difference */
      std::set_difference(fi[xag.node_to_index(chs[1].node)].begin(), fi[xag.node_to_index(chs[1].node)].end(), 
        fi[xag.node_to_index( chs[0].node )].begin(), fi[xag.node_to_index( chs[0].node )].end(), std::inserter(chs[1].target, chs[1].target.begin()));
      
      /* the first may be included */
      if( first_cone_included_in_second(fi[xag.node_to_index(chs[0].node)], fi[xag.node_to_index(chs[1].node)] ) )
      {
        /* add the top of the cone to the second */
        chs[1].target.push_back( chs[0].node );
        chs[1].leaves = chs[1].target;

        /* anything can be chosen as target */ 
        chs[0].target = fi[xag.node_to_index( chs[0].node )];
      }

    }
    else
    {
      if ( xag.is_xor( fanins[1] ) )
        std::reverse(fanins.begin(), fanins.end());

      std::set_difference(fi[xag.node_to_index(fanins[0])].begin(), fi[xag.node_to_index(fanins[0])].end(), 
        fi[xag.node_to_index(fanins[1])].begin(), fi[xag.node_to_index(fanins[1])].end(), std::inserter(chs[0].target, chs[0].target.begin()) );
    }

    return chs;
  }

  
    
inline std::vector<std::pair<mockturtle::node<mockturtle::xag_network>, mapping_strategy_action>> gen_steps( mockturtle::node<mockturtle::xag_network> node, std::vector<action_sets>& chs, bool compute)
{
  std::vector<std::pair<mockturtle::node<mockturtle::xag_network>, mapping_strategy_action>> comp_steps;
  for(auto ch : chs)
  {
    if (ch.leaves.size() >= 1){
      if ( !ch.target.empty() )
      {
        comp_steps.push_back( {ch.node, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves}} );
      }
      else 
      {
        comp_steps.push_back( {ch.node, compute_action{ ch.leaves, std::nullopt }} );
      }
    }
  }
  

  if(compute)
    comp_steps.push_back( {node, compute_action{}} );
  else 
  {
    comp_steps.push_back( {node, uncompute_action{}} );
  }


  std::reverse( chs.begin(), chs.end() );

  for(auto ch : chs)
  {
    if (ch.leaves.size() >= 1){
      if ( !ch.target.empty() )
      {
        comp_steps.push_back( {ch.node, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves}} );
      }
      else 
      {
        comp_steps.push_back( {ch.node, compute_action{ ch.leaves, std::nullopt}} );
      }
    }
  }

  return comp_steps;
}

/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It exploits two main facts:

    1.  XORs are relatively cheap to be implemented in fault tolerant quantum computing
    2.  Toffoli gates used to implement AND nodes can be uncomputed using 0 T gates

    Details can be found in :cite:`MSC19`.
  \endverbatim
*/
class xag_mapping_strategy : public mapping_strategy<mockturtle::xag_network>
{

  std::vector<std::vector<uint32_t>> fi;
  mockturtle::xag_network xag;

public:
  bool compute_steps( mockturtle::xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};

    std::vector<mockturtle::node<mockturtle::xag_network>> drivers;
    xag.foreach_po( [&]( auto const& f ) { drivers.push_back( xag.get_node( f ) ); } );
                                                                                   

    auto it = steps().begin();


    xag.foreach_node( [&]( auto node ) {
      
      auto fi = compute_fi( node, xag );



      if ( xag.is_and( node ) || std::find( drivers.begin(), drivers.end(), node ) != drivers.end() )
      {
        auto chs = get_fi_target(  node, xag, fi );

        if ( xag.is_and( node ))
        {
        
          /* compute step */
          auto cc = gen_steps( node , chs,  true);
          it = steps().insert( it, cc.begin(), cc.end() );
          it = it + cc.size();

          if ( std::find( drivers.begin(), drivers.end(), node ) == drivers.end() )
          {
            auto uc = gen_steps( node , chs, false);
            it = steps().insert( it, uc.begin(), uc.end() );
          }
        }
        /* node is an XOR output */
        else 
        {
          auto xc = gen_steps( node, chs, true);
          it = steps().insert( it, xc.begin(), xc.end() );

        }
      }

    } );

    return true;
  }
};


/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It creates an abstract graph from the XAG, each box node in the abstract graph corresponds to 
    AND nodes and its linear transitive fanin cones.
    Pebbling is played on the abstract graph.
  \endverbatim
*/
class xag_pebbling_mapping_strategy : public mapping_strategy<mockturtle::xag_network>
{
  using StepsXAG = std::vector<std::pair<mockturtle::xag_network::node, mapping_strategy_action>>;
  using StepsABS = std::vector<std::pair<abstract_network::node, mapping_strategy_action>>;

  using SolverType = z3_pebble_solver<abstract_network>;
 
  StepsXAG get_box_steps(StepsABS const& store_steps)
  {
    StepsXAG xag_steps;
    for(auto step : store_steps)
    {
      auto box_node = step.first;
      auto action = step.second;

      auto chs = box_to_action[box_node];

      auto cc = gen_steps( chs[0].node, chs, std::holds_alternative<compute_action>( action ));
      xag_steps.insert(xag_steps.end(), cc.begin(), cc.end());
    }

    return xag_steps;
  }

  abstract_network build_box_network(pebbling_view<mockturtle::xag_network> const& xag)
  {

    std::unordered_map<mockturtle::xag_network::node, abstract_network::signal> xag_to_box;
    auto box_ntk = abstract_network();

    xag.foreach_pi([&] (auto pi)
    {
      xag_to_box[pi] = box_ntk.create_pi(); 
    });

    std::vector<mockturtle::node<mockturtle::xag_network>> drivers;
    xag.foreach_po( [&]( auto const& f ) { drivers.push_back( xag.get_node( f ) ); } );

    xag.foreach_node([&] (auto node)
    {

      if(xag.is_and(node))
      {
        /* collect all input signals of box */
        std::vector<abstract_network::signal> box_ins;
        auto fi = compute_fi( node, xag );

        auto chs = get_fi_target(  node, xag, fi );

        for( auto fi_box : chs)
        {
          for (auto l : fi_box.leaves)
          {
            box_ins.push_back(xag_to_box[l]);
          }
        }

        auto box_node_s = box_ntk.create_node(box_ins);
        xag_to_box[node] = box_node_s;
        box_to_action[box_node_s] = chs;
        
        if(std::find(drivers.begin(), drivers.end(), node) != drivers.end())
        {
          box_ntk.create_po(box_node_s);
        }
      }
      //TODO : I am still assuming that all outputs are driven by an AND
      //TODO : inversions
    });

    return box_ntk;
  }

  pebbling_mapping_strategy_params ps;
  std::unordered_map<abstract_network::node, std::vector<action_sets>> box_to_action;

public: 
  
  xag_pebbling_mapping_strategy( pebbling_mapping_strategy_params const& ps = {} )
  : ps( ps ) {}

  bool compute_steps( mockturtle::xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};

    /* build the abstract network */
    abstract_network box_ntk = build_box_network(xag);

    StepsABS store_steps;

    auto limit = ps.pebble_limit;

    while ( true )
    {
      SolverType solver( box_ntk, limit, ps.conflict_limit );

      solver.init();

      typename SolverType::result result;

      do
      {
        if ( solver.current_step() >= ps.max_steps )
        {
          result = solver.unknown();
          break;
        }

        solver.add_step();
        result = solver.solve(); 
      } while ( result == solver.unsat() );

      if ( result == solver.unknown() )
      {
          return false;
      }
      else if ( result == solver.sat() )
      {
        store_steps = solver.extract_result();
      }

      if ( store_steps.empty() )
        return false;
    }

    this -> steps() = get_box_steps(store_steps);
    return true;
  }
};


} // namespace caterpillar