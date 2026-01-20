#include <iostream>
#include <string>
#include <numbers>
#include "turtlelib/angle.hpp"

/**
 * @brief Simple program to convert and normalize angles between degrees and radians.
 * * This program prompts the user for an angle and its unit (deg or rad),
 * then outputs the normalized version of that angle in both units.
 * It continues until EOF (CTRL-D) is reached.
 */
int main()
{
  double input_val = 0.0;
  std::string unit = "";

  while (true) {
        // Prompt user for input
    std::cout << "Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)\n";

        // Attempt to read the double and the string
    if (!(std::cin >> input_val >> unit)) {
            // Check if loop should terminate due to CTRL-D
      if (std::cin.eof()) {
        break;
      }

            // Handle malformed input (e.g., user entered a letter instead of a number)
      std::cout << "Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";
      std::cin.clear();       // Clear error flags
      std::string discard;
      std::getline(std::cin, discard);       // Discard the rest of the line
      continue;
    }

    if (unit == "deg") {
            // 1. Convert input degrees to radians
      double rad = turtlelib::deg2rad(input_val);
            // 2. Normalize the radian value to (-PI, PI]
      double norm_rad = turtlelib::normalize_angle(rad);
            // 3. Convert normalized radians back to degrees for output
      double norm_deg = turtlelib::rad2deg(norm_rad);

      std::cout << input_val << " deg is " << norm_deg << " deg and " << norm_rad << " rad.\n";
    } else if (unit == "rad") {
            // 1. Normalize the input radian value to (-PI, PI]
      double norm_rad = turtlelib::normalize_angle(input_val);
            // 2. Convert normalized radians to degrees
      double norm_deg = turtlelib::rad2deg(norm_rad);

      std::cout << input_val << " rad is " << norm_rad << " rad and " << norm_deg << " deg.\n";
    } else {
            // Handle valid double but invalid unit string
      std::cout << "Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";
    }
  }

  return 0;
}
