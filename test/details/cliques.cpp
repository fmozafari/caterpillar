#include <catch.hpp>
#include <iostream>

#include <caterpillar/details/bron-kerbosch.hpp>
#include <caterpillar/details/bron-kerbosch_utils.hpp>



using namespace caterpillar::detail;
using namespace std::placeholders;


TEST_CASE ("mytest", "mytestenum")
{
  Graph <int> P;
  P.push_front({1});
  P.push_front({2});
  P.push_front({3});
  P.push_front({4});
  P.push_front({5});
  P.push_front({6});

  edge <int> (P, 1, 2);
  edge <int> (P, 1, 3);
  edge <int> (P, 3, 4);
  edge <int> (P, 2, 5);
  edge <int> (P, 4, 5);
  edge <int> (P, 5, 6);

  Solution<int> solution;

  solve <int> ({ {} }, P, { {} }, std::bind (aggregator <int>, std::ref (solution), _1, _2, _3) );

  auto count = 0;
  for(auto R : solution){ count++; }

  CHECK(count == 6);


}