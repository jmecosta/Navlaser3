/*
    Copyright (c) 2011 <copyright holder> <email>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
*/


#include "config_extractorGTest.h"

config_extractorGTest::config_extractorGTest()
{

}

config_extractorGTest::~config_extractorGTest()
{

}

void config_extractorGTest::SetUp() {
  
  
}
void config_extractorGTest::TearDown() {
  
  
}

// Definition of the Tests
TEST(config_extractorTest, setCornerEstimationMethodReturnBool) {
  config_extractor c;
      
  ASSERT_TRUE(c.setCornerEstimationMethod(CORNERRAMSAC,0.1));
  ASSERT_EQ(c.CornerEstimationMethod,CORNERRAMSAC);
  ASSERT_EQ(c.RANSAC_FACTOR,0.2);

}
TEST(config_extractorTest, setLineEstimationMethodReturnBool) {
  config_extractor c;
      
  ASSERT_TRUE(c.setLineEstimationMethod(LINELEASTSQUAREMETHOD));
  ASSERT_EQ(c.LineEstimationMethod,LINELEASTSQUAREMETHOD);
  
}

