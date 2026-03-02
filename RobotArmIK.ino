void updateServos() {
  float theta1, theta2;
  inverseKinematics(x_d, y_d, theta1, theta2);

  // Print the angles to the serial monitor
  Serial.print("Theta1: ");
  Serial.print(theta1);
  Serial.println(" degrees");

  Serial.print("Theta2: ");
  Serial.print(theta2);
  Serial.println(" degrees");

  // Map the angles to servo positions (0 to 180 degrees)
  int Apos1 = map(theta1, -90, 90, 25, 170); // Adjust the mapping as needed
  int Apos2 = map(theta2, -90, 90, 165, 95); // Adjust the mapping as needed


  // Move the servos to the calculated positions
  shoulder.write(Apos1);
  elbow.write(Apos2);
}

void inverseKinematics(float x_d, float y_d, float &theta1, float &theta2) {
  // Calculate the distance from the base to the end effector position
  float r = sqrt(x_d * x_d + y_d * y_d);

  // Calculate phi
  float phi = atan2(y_d, x_d);

  // Calculate theta2 using the cosine rule
  float cos_theta2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (cos_theta2 < -1 || cos_theta2 > 1) {
    Serial.println("The position is out of reach for the robot arm.");
    theta1 = theta2 = NAN;
    return;
  }

  theta2 = acos(cos_theta2);

  // Calculate theta1
  float sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);
  theta1 = phi - atan2(L2 * sin_theta2, L1 + L2 * cos_theta2);

  // Convert radians to degrees
  theta1 = degrees(theta1);
  theta2 = degrees(theta2);
}
