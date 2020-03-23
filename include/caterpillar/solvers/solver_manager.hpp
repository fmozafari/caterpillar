#pragma once


#include <mockturtle/utils/progress_bar.hpp>
#include <caterpillar/synthesis/strategies/action.hpp>

namespace caterpillar
{

struct pebbling_mapping_strategy_params
{
  /*! \brief Show progress bar. */
  bool progress{false};

  /*! \brief Maximum number of pebbles to use, if supported by mapping strategy (0 means no limit). */
  uint32_t pebble_limit{0u};

  /*! \brief Maximum number of steps */
  uint32_t max_steps{100000};

  /*! \brief max_weight */
  uint32_t max_weight{0u};

  /*! \brief Conflict limit for the SAT solver (0 means no limit). */
  uint32_t conflict_limit{0u};

  /*! \brief Increment pebble numbers, if a failure occurs. */
  bool increment_on_failure{false};

  /*! \brief Decrement pebble numbers, if satisfiable. */
  bool decrement_on_success{false};

};

template<typename Ntk>
using Steps = std::vector<std::pair<typename Ntk::node, mapping_strategy_action>>;

template <typename Solver, typename Ntk>
inline Steps<Ntk> pebble (Ntk ntk, pebbling_mapping_strategy_params const& ps = {})
{

  assert( !ps.decrement_on_success || !ps.increment_on_failure );

  auto limit = ps.pebble_limit;
  
  Steps<Ntk> steps;
  while ( true )
  {
    Solver solver( ntk, limit, ps.conflict_limit, ps.max_weight );

    solver.init();

    mockturtle::progress_bar bar( 100, "|{0}| current step = {1}", ps.progress );

    typename Solver::result result;

    do
    {
      if ( solver.current_step() >= ps.max_steps )
      {
        result = solver.unknown();
        break;
      }

      bar( std::min<uint32_t>( solver.current_step(), 100 ), solver.current_step() );

      solver.add_step();
      result = solver.solve(); 
    } while ( result == solver.unsat() );

    if ( result == solver.unknown() )
    {
      if ( ps.increment_on_failure )
      {
        limit++;
        continue;
      }
      else if ( !ps.decrement_on_success )
        return steps;
    }
    else if ( result == solver.sat() )
    {
      steps = solver.extract_result();
      if ( ps.decrement_on_success )
      {
        limit--;
        continue;
      }
    }

    return steps;
  }

}

}//caterpillar