/*-------------------------------------------------------------------------------------------------
| This file is distributed under the MIT License.
| See accompanying file /LICENSE for details.
| Author(s): Giulia Meuli
*------------------------------------------------------------------------------------------------*/

/*! 
  \file pebbling_view.hpp
  \brief Implements weights for networks
*/
#include <mockturtle/views/immutable_view.hpp>

#include <iostream>
namespace caterpillar
{

/*! 

**Required network functions:**
  - num_gates,
  - num_pis

*/

template<typename Ntk>
class pebbling_view : public mockturtle::immutable_view<Ntk>
{
public:
  using node = typename Ntk::node;
  pebbling_view( Ntk const& network )
      : mockturtle::immutable_view<Ntk>( network )
  {
    static_assert( mockturtle::is_network_type_v<Ntk>, "Ntk is not a network type" );
    static_assert( mockturtle::has_num_gates_v<Ntk>, "Ntk does not implement the num_gates method" );
    static_assert( mockturtle::has_num_pis_v<Ntk>, "Ntk does not implement the num_pis method" );

    _weights.resize(this ->num_gates(), 1);

  }

  uint32_t get_weight (const node n) const
  {
    return _weights[this->node_to_index( n - this->num_pis() - 1 )];
  }

  void set_weight (const node n, uint32_t w)
  {
    _weights[this->node_to_index( n - this->num_pis() - 1 )] = w;
  }

private:
  std::vector<uint32_t> _weights;
};

 } // namespace caterpillar