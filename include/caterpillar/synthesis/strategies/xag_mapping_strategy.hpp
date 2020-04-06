/*-------------------------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*------------------------------------------------------------------------------------------------*/

#pragma once

#include "mapping_strategy.hpp"
#include <caterpillar/solvers/solver_manager.hpp>
#include <caterpillar/structures/stg_gate.hpp>
#include <caterpillar/structures/pebbling_view.hpp>
#include <caterpillar/structures/abstract_network.hpp>
#include <caterpillar/synthesis/strategies/pebbling_mapping_strategy.hpp>
#include <caterpillar/solvers/z3_solver.hpp>
#include <mockturtle/networks/xag.hpp>
#include <mockturtle/views/topo_view.hpp>
#include <mockturtle/views/depth_view.hpp>
#include <tweedledum/networks/netlist.hpp>

#include <algorithm>


namespace caterpillar
{

using node_t = mockturtle::xag_network::node;
using steps_xag_t = std::vector<std::pair<node_t, mapping_strategy_action>>;
#ifdef USE_Z3
using steps_abs_t = std::vector<std::pair<abstract_network::node, mapping_strategy_action>>;
using SolverType = z3_pebble_solver<abstract_network>;
#endif
struct action_sets
{
  node_t node;
  std::vector<uint32_t> target;
  std::vector<uint32_t> leaves;

  action_sets( node_t node, std::vector<uint32_t> leaves, std::vector<uint32_t> target = {} )
      : node(node), target(target), leaves(leaves) {};
};

inline std::vector<uint32_t> sym_diff(std::vector<uint32_t> first, std::vector<uint32_t> second)
{
  std::vector<uint32_t> diff;
  std::set_symmetric_difference( first.begin(), first.end(), second.begin(), second.end(), std::back_inserter(diff) );
  return diff;
}

inline bool is_included(std::vector<uint32_t> first, std::vector<uint32_t> second)
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


inline void update_fi( node_t node, mockturtle::xag_network const& xag, std::vector<std::vector<uint32_t>>& fi, std::vector<node_t> const& drivers )
{
  if ( xag.is_and( node ) || xag.is_pi(node) || (std::find(drivers.begin(), drivers.end(), node) != drivers.end()))
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
}

inline  std::vector<action_sets> get_cones( node_t node, mockturtle::xag_network const& xag, std::vector<std::vector<uint32_t>>& fi )
{
  std::vector<action_sets> cones; 

  xag.foreach_fanin( node, [&]( auto si ) {
    auto fanin = xag.get_node( si );

    auto set = action_sets(fanin, fi[xag.node_to_index( fanin )] ) ; 
    cones.push_back( set ); 
  } );

  assert( cones.size() == 2 );

  // if one fanin is AND or PI or PO symply delete f
  if ( (cones[0].leaves.size() == 1) != (cones[1].leaves.size() == 1) )
  {
    if ( xag.is_xor( cones[1].node ) )
      std::reverse(cones.begin(), cones.end());

    cones[0].target = cones[0].leaves;

    /* remove the single leaf if there is overlap */
    auto it = std::find(cones[0].target.begin(), cones[0].target.end(), cones[1].leaves[0]);
    if (it != cones[0].target.end())
      cones[0].target.erase( it );
  }
  else 
  {
    if ( is_included(fi[xag.node_to_index(cones[1].node)], fi[xag.node_to_index(cones[0].node)]) )
      std::reverse(cones.begin(), cones.end());

    auto left = xag.node_to_index(cones[0].node);
    auto right = xag.node_to_index(cones[1].node);

    /* search a target for first */
    /* empty if left is included */
    std::set_difference(fi[left].begin(), fi[left].end(), 
      fi[right].begin(), fi[right].end(), std::inserter(cones[0].target, cones[0].target.begin()));

    /* set difference */
    std::set_difference(fi[right].begin(), fi[right].end(), 
      fi[left].begin(), fi[left].end(), std::inserter(cones[1].target, cones[1].target.begin()));
    
    /* the first may be included */
    if( is_included(fi[left], fi[right] ) )
    {
      /* add the top of the cone to the right */
      cones[1].target.push_back( cones[0].node ); 
      cones[1].leaves = cones[1].target;

      /* anything can be chosen as target */ 
      cones[0].target = fi[left];
    }
  }

  return cones;
}

inline steps_xag_t gen_steps( node_t node, std::vector<action_sets>& cones, bool compute, std::vector<uint32_t> const& cps = {})
{
  steps_xag_t comp_steps;

  for(auto ch : cones)
  {

    if (ch.leaves.size() > 1){
      if ( !ch.target.empty() )
      {
        if (cps.empty())
          comp_steps.push_back( {ch.node, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves, std::nullopt}} );
        else 
          comp_steps.push_back( {ch.node, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves, cps}} );
      }
      else 
      {
        if (cps.empty())
          comp_steps.push_back( {ch.node, compute_action{ ch.leaves, std::nullopt, std::nullopt}} );
        else 
          comp_steps.push_back( {ch.node, compute_action{ ch.leaves, cps, std::nullopt}} );

      }
    }
  }
  
  if(compute)
  {
    if (cps.empty())
      comp_steps.push_back( {node, compute_action{}} );
    else
      comp_steps.push_back( {node, compute_action{std::nullopt, cps, std::nullopt}} );
  }
  else 
  {
    comp_steps.push_back( {node, uncompute_action{}} );
  }


  std::reverse( cones.begin(), cones.end() );

  for(auto ch : cones)
  {
    if (ch.leaves.size() > 1){
      if ( !ch.target.empty() )
      {
        if (cps.empty())
          comp_steps.push_back( {ch.node, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves, std::nullopt}} );
        else 
          comp_steps.push_back( {ch.node, compute_inplace_action{static_cast<uint32_t>( ch.target[0] ), ch.leaves, cps}} );
      }
      else 
      {
        if (cps.empty())
          comp_steps.push_back( {ch.node, compute_action{ ch.leaves, std::nullopt, std::nullopt}} );
        else 
          comp_steps.push_back( {ch.node, compute_action{ ch.leaves, cps, std::nullopt}} );

      }
    }
  }
  for(auto& cp : cps)
    comp_steps.push_back({cp, uncompute_copy_action{}});
  
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

public:
  bool compute_steps( mockturtle::xag_network const& ntk ) override
  {
    mockturtle::topo_view xag {ntk};
    std::vector<std::vector<uint32_t>> fi (xag.size());

    std::vector<node_t> drivers;
    xag.foreach_po( [&]( auto const& f ) { drivers.push_back( xag.get_node( f ) ); } );
                                                                                   

    auto it = steps().begin();


    xag.foreach_node( [&]( auto node ) {
      
      update_fi( node, xag, fi, drivers );

      if ( xag.is_and( node ) || std::find( drivers.begin(), drivers.end(), node ) != drivers.end() )
      {
        auto cones = get_cones(  node, xag, fi );

        if ( xag.is_and( node ))
        {
          /* compute step */
          auto cc = gen_steps( node , cones,  true);
          it = steps().insert( it, cc.begin(), cc.end() );
          it = it + cc.size();

          if ( std::find( drivers.begin(), drivers.end(), node ) == drivers.end()  )
          { 
            auto uc = gen_steps( node , cones, false);
            it = steps().insert( it, uc.begin(), uc.end() );
          }
        }
        /* node is an XOR output */
        else 
        {
          auto xc = gen_steps( node, cones, true);
          it = steps().insert( it, xc.begin(), xc.end() );

        }
      }

    } );

    return true;
  }
};

#ifdef USE_Z3
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

 
  steps_xag_t get_box_steps(steps_abs_t const& store_steps)
  {
    steps_xag_t xag_steps;
    for(auto step : store_steps)
    {
      auto box_node = step.first;
      auto action = step.second;

      auto [node, cones] = box_to_action[box_node];

      auto cc = gen_steps( node, cones, std::holds_alternative<compute_action>( action ));
      xag_steps.insert(xag_steps.end(), cc.begin(), cc.end());
    }

    return xag_steps;
  }

  abstract_network build_box_network(mockturtle::xag_network const& xag, bool use_w = false)
  {
    abstract_network box_ntk;

    std::unordered_map<node_t, abstract_network::signal> xag_to_box;
    std::vector<std::vector<uint32_t>> fi (xag.size());


    xag.foreach_pi([&] (auto pi)
    {
      xag_to_box[pi] = box_ntk.create_pi(); 
    });

    std::vector<node_t> drivers;
    xag.foreach_po( [&]( auto const& f ) { drivers.push_back( xag.get_node( f ) ); } );

    xag.foreach_node([&] (auto node)
    {
      update_fi( node, xag, fi, drivers );

      if(xag.is_and(node) || std::find(drivers.begin(), drivers.end(), node) != drivers.end())
      {
        /* collect all the input signals of the box */
        std::vector<abstract_network::signal> box_ins;

        auto cones = get_cones(  node, xag, fi );
        for( auto c : cones)
        {
          for (auto l : c.leaves)
          {
            box_ins.push_back(xag_to_box[l]);
          }
        }
        
        
        auto box_node_s = use_w ? box_ntk.create_node(box_ins, (int)box_ins.size()/2) : box_ntk.create_node(box_ins);
        auto box_node = box_ntk.get_node(box_node_s);

        xag_to_box[node] = box_node_s;

        box_to_action[box_node] = {node, cones};
       
        
        if(std::find(drivers.begin(), drivers.end(), node) != drivers.end())
        {
          box_ntk.create_po(box_node_s);
        }
      }
    });

    return box_ntk;
  }

  pebbling_mapping_strategy_params ps;
  std::unordered_map<abstract_network::node, std::pair<node_t, std::vector<action_sets>> > box_to_action;

public: 
  
  xag_pebbling_mapping_strategy( pebbling_mapping_strategy_params const& ps = {} )
  : ps( ps ) {}

  bool compute_steps( mockturtle::xag_network const& ntk ) override
  {
    using solver_t = z3_pebble_solver<abstract_network>;
    mockturtle::topo_view xag {ntk};
    steps_abs_t store_steps;
    

    if(ps.progress) std::cout << "[i]  Generate box network... \n";
    auto box_ntk = build_box_network(xag, ps.max_weight != 0);

    store_steps = pebble<solver_t, abstract_network> (box_ntk, ps);

    if ( store_steps.empty() )
      return false;


    if(ps.progress) std::cout << "[i] Extract the XAG strategy ...\n";

    this -> steps() = get_box_steps(store_steps);
    return true;
  }
};
#endif



/*!
  \verbatim embed:rst
    This strategy is dedicated to XAG graphs and fault tolerant quantum computing.
    It creates an abstract graph from the XAG, each box node in the abstract graph corresponds to 
    AND nodes and its linear transitive fanin cones.
    Pebbling is played on the abstract graph.
  \endverbatim
*/
class xag_low_depth_mapping_strategy : public mapping_strategy<mockturtle::xag_network>
{
  template<class T>
  T concat (T f, T const& s)
  {
    f.insert(f.end(), s.begin(), s.end());
    return f;
  }
public: 
  
  bool compute_steps( mockturtle::xag_network const& ntk ) override
  {
    // the strategy proceeds in topological order and level by level
    mockturtle::topo_view xag_t {ntk};
    mockturtle::depth_view xag {xag_t};

    std::vector<std::vector<uint32_t>> fi (xag.size());

    std::vector<node_t> drivers;
    xag.foreach_po( [&]( auto const& f ) { drivers.push_back( xag.get_node( f ) ); } );

    std::vector<std::vector<node_t>> levels (xag.depth());
                                                                 
    xag.foreach_node( [&]( auto n ) {
      update_fi( n, xag, fi, drivers );

      if( xag.is_and(n) ) 
        levels[xag.m_level(n)-1].push_back(n);
      else if( std::find( drivers.begin(), drivers.end(), n ) != drivers.end() ) 
        levels[xag.level(n)-1].push_back(n);
      
    });
    
    auto it = steps().begin();

    for(auto lvl : levels){ if(lvl.size() != 0)
    {

      std::map< node_t, std::pair<std::vector<action_sets>, std::vector<uint32_t>> > node_and_action;
      
      std::vector<uint32_t> used;
      std::vector<uint32_t> to_be_copied;

      for(auto n : lvl)
      {
        auto cones = get_cones(n, xag, fi);

        if(xag.is_and(n))
        {
          auto tot_l = concat(cones[0].leaves, cones[1].leaves);
          std::vector<uint32_t> cp;
          for (auto l : tot_l)
          {
            if(std::find(used.begin(), used.end(), l ) != used.end())
            {
              to_be_copied.push_back(l);
              cp.push_back(l);
            }
            used.push_back(l);
          }
          node_and_action[n] = {cones, cp};
        }
        else //xor outputs do not need to use copies
          node_and_action[n] = {cones, {}};
      }

      for(auto l : to_be_copied)
      {
        it = steps().insert(it, {l, compute_copy_action{}} );
        it = it + 1;
      }
      for (auto n : lvl)
      {

        auto cc = gen_steps(n, node_and_action[n].first, true, node_and_action[n].second);

        it = steps().insert(it, cc.begin(), cc.end());
        it = it + cc.size();        


        if ( std::find( drivers.begin(), drivers.end(), n ) == drivers.end()  )
        { 
          auto uc = gen_steps( n , node_and_action[n].first, false);
          it = steps().insert( it, uc.begin(), uc.end() );
        }
        
      }
    }
    }
    return true;
  }
};
} // namespace caterpillar