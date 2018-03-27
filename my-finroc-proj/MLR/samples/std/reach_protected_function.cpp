


/*how to reach the protected function in c++*/


/*stackoverflow Q& A*/

/*impl for Verify or Vilify ;)*/

class Base
{

private:
  int MyPrivateInt;
protected:
  int MyProtectedInt;
public:
  int MyPublicInt;
}

class Derived : Base
{
public:
  int foo1()
  {
    return MyPrivateInt; // Won't compile!
  }
  int foo2()
  {
    return MyProtectedInt; // OK
  }
  int foo3()
  {
    return MyPublicInt; // OK
  }
};

class Unrelated
{
private:
  Base B;
public:
  int foo1()
  {
    return B.MyPrivateInt; // Won't compile!
  }
  int foo2()
  {
    return B.MyProtectedInt; // Won't compile
  }
  int foo3()
  {
    return B.MyPublicInt; // OK
  }
};





//////////////////////////////////////////////////////////////
/*
down vote
It all depends on what you want to do, and what you want the derived classes to be able to see.
*/

class A
{
private:
  int _privInt = 0;
  int privFunc()
  {
    return 0;
  }
  virtual int privVirtFunc()
  {
    return 0;
  }
protected:
  int _protInt = 0;
  int protFunc()
  {
    return 0;
  }
public:
  int _publInt = 0;
  int publFunc()
  {
    return privVirtFunc();
  }
};

class B : public A
{
private:
  virtual int privVirtFunc()
  {
    return 1;
  }
public:
  void func()
  {
    _privInt = 1; // wont work
    _protInt = 1; // will work
    _publInt = 1; // will work
    privFunc(); // wont work
    privVirtFunc(); // wont work
    protFunc(); // will work
    publFunc(); // will return 1 since it's overridden in this class
  };
