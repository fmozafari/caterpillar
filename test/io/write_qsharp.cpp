#include <catch.hpp>
#include <mockturtle/networks/xag.hpp>
#include <tweedledum/networks/netlist.hpp>
#include <tweedledum/io/write_unicode.hpp>
#include <caterpillar/synthesis/lhrs.hpp>
#include <caterpillar/structures/stg_gate.hpp>
#include <caterpillar/synthesis/strategies/xag_mapping_strategy.hpp>
#include <caterpillar/io/write_qsharp.hpp>

TEST_CASE("write xag in Q#", "[write xag in Q#]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  auto xag = xag_network();

  auto a = xag.create_pi();
  auto b = xag.create_pi();
  auto c = xag.create_pi();
  auto d = xag.create_and( a, b );
  auto e = xag.create_xor( d, c );
  xag.create_po( e );

  netlist<stg_gate> rnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( rnet, xag, strategy, {}, ps, &st );

  std::ostringstream out;
  write_qsharp(rnet, out, "Test1");

  CHECK( out.str() == "operation Test1(): Unit\n"
                      "{\n"
                      "  using (qs = Qubit[5])\n"
                      "  {\n"
                      "    AND(qs[0], qs[1], qs[3]);\n"
                      "    CNOT(qs[3], qs[4] );\n"
                      "    CNOT(qs[2], qs[4] );\n"
                      "    Adjoint AND(qs[0], qs[1], qs[3]);\n"
                      "\n"
                      "  }\n"
                      "}\n" );
}


TEST_CASE("write xag with negations in Q#", "[xag with negations in Q#]")
{
  using namespace caterpillar;
  using namespace mockturtle;
  using namespace tweedledum;

  auto xag = xag_network();

  auto x0 = xag.create_pi();
  auto x3 = xag.create_pi();
  auto x4 = xag.create_pi();
  auto x5 = xag.create_pi();
  auto x6 = xag.create_pi();
  auto x7 = xag.create_not(x6);
  auto n10 = xag.create_xor(x7, x0);
  auto n9 = xag.create_xor(x5, x3);
  auto n16 = xag.create_xor(n10, n9);
  auto n20 = xag.create_xor(n16, x4);
  auto n21 = xag.create_xor(n20, x5);
  auto n32 = xag.create_and(n16, n21);
  auto n33 = xag.create_not(n32);
  xag.create_po(n33);

  netlist<stg_gate> rnet;

  logic_network_synthesis_params ps;
  logic_network_synthesis_stats st;
  ps.verbose = false;

  xag_mapping_strategy strategy;
  logic_network_synthesis( rnet, xag, strategy, {}, ps, &st );

  std::ostringstream out;
  write_qsharp(rnet, out, "Test2");
  CHECK(out.str() ==  "operation Test2(): Unit\n"
                      "{\n"
                      "  using (qs = Qubit[6])\n"
                      "  {\n"
                      "    CNOT(qs[0], qs[3] );\n"
                      "    CNOT(qs[1], qs[3] );\n"
                      "    CNOT(qs[4], qs[3] );\n"
                      "    CNOT(qs[0], qs[2] );\n"
                      "    CNOT(qs[1], qs[2] );\n"
                      "    CNOT(qs[4], qs[2] );\n"
                      "    AND(qs[3], qs[2], qs[5]);\n"
                      "    CNOT(qs[0], qs[2] );\n"
                      "    CNOT(qs[1], qs[2] );\n"
                      "    CNOT(qs[4], qs[2] );\n"
                      "    CNOT(qs[0], qs[3] );\n"
                      "    CNOT(qs[1], qs[3] );\n"
                      "    CNOT(qs[4], qs[3] );\n"
                      "    NOT(qs[5]);\n"
                      "\n"
                      "  }\n"
                      "}\n");

}