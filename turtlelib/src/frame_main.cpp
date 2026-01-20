#include <iostream>
#include <fstream>
#include <string>
#include <format>
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

using namespace turtlelib;

/**
 * \brief Main function for the frame transformation visualization exercise.
 * Performs SE(2) calculations and outputs an SVG file to /tmp/frames.svg.
 */
int main()
{
  Transform2D T_ab, T_bc;

    // 1. Prompt for transforms (to stderr)
  std::cerr << "Enter transform T_ab:" << std::endl;
  if (!(std::cin >> T_ab)) {return 1;}

  std::cerr << "Enter transform T_bc:" << std::endl;
  if (!(std::cin >> T_bc)) {return 1;}

    // 2. Compute related transforms
  Transform2D T_ba = T_ab.inv();
  Transform2D T_cb = T_bc.inv();
  Transform2D T_ac = T_ab * T_bc;
  Transform2D T_ca = T_ac.inv();

    // Output calculated transforms to stdout using std::format
  std::cout << std::format("T_ab: {}\n", T_ab);
  std::cout << std::format("T_ba: {}\n", T_ba);
  std::cout << std::format("T_bc: {}\n", T_bc);
  std::cout << std::format("T_cb: {}\n", T_cb);
  std::cout << std::format("T_ac: {}\n", T_ac);
  std::cout << std::format("T_ca: {}\n", T_ca);

    // Initialize SVG and draw coordinate frames
  Svg svg;
  svg.draw(Transform2D(), "a");   // Frame {a} at identity
  svg.draw(T_ab, "b");            // Frame {b} relative to {a}
  svg.draw(T_ac, "c");            // Frame {c} relative to {a}

    // 3. Point Logic
  Point2D p_a;
  std::cerr << "Enter point p_a:" << std::endl;
  if (!(std::cin >> p_a)) {return 1;}

    // Compute point location in other frames
  Point2D p_b = T_ba(p_a);
  Point2D p_c = T_ca(p_a);

    // Output point locations to stdout
  std::cout << std::format("p_a: {}\n", p_a);
  std::cout << std::format("p_b: {}\n", p_b);
  std::cout << std::format("p_c: {}\n", p_c);

    // Draw point in different frames (mapped back to {a} for SVG)
  svg.draw(p_a, "purple");              // p_a directly
  svg.draw(T_ab(p_b), "brown");         // p_b through T_ab
  svg.draw(T_ac(p_c), "orange");        // p_c through T_ac

    // 4. Vector Logic
  Vector2D v_b;
  std::cerr << "Enter vector v_b:" << std::endl;
  if (!(std::cin >> v_b)) {return 1;}

    // Normalize v_b
  Vector2D v_b_hat = normalize(v_b);
  std::cout << std::format("v_b_hat: {}\n", v_b_hat);

    // Draw v_b_hat with tail at (0,0) in {b}, expressed in {a}
  svg.draw(T_ab(Point2D{0, 0}), T_ab(v_b_hat), "brown");

    // Express v in frames {a} and {c}
  Vector2D v_a_hat = T_ab(v_b_hat);
  Vector2D v_c_hat = T_cb(v_b_hat);
  std::cout << std::format("v_a_hat: {}\n", v_a_hat);
  std::cout << std::format("v_c_hat: {}\n", v_c_hat);

    // Draw v_a_hat with tail at (0,0) in {a}
  svg.draw(Point2D{0, 0}, v_a_hat, "black");

    // Draw v_b_hat with tail at p_b in frame {b}, expressed in {a}
  svg.draw(T_ab(p_b), T_ab(v_b_hat), "purple");

    // Draw v_c_hat with tail at p_c in frame {c}, expressed in {a}
  svg.draw(T_ac(p_c), T_ac(v_c_hat), "orange");

    // 5. Output to SVG file
  std::ofstream out_file("/tmp/frames.svg");
  if (out_file.is_open()) {
    out_file << svg.write();
    std::cerr << "SVG output written to /tmp/frames.svg" << std::endl;
  } else {
    std::cerr << "Error: Could not open /tmp/frames.svg for writing" << std::endl;
    return 1;
  }

  return 0;
}
