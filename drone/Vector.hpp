class Vector
{
  public:
    float x;
    float y;
    float z;
    Vector(){};
    Vector(float _x, float _y, float _z)
    {
      x = _x;
      y = _y;
      z = _z;
    }
    void print_vector()
    {
      Serial.println();
      Serial.print("X = ");
      Serial.print(this->x);
      Serial.print("\t");
      Serial.print("Y = ");
      Serial.print(this->y);
    }
};
