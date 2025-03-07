//***************************************************************************
// apps/testing/cxx/cxxsize/exception.cxx
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed to the Apache Software Foundation (ASF) under one or more
// contributor license agreements.  See the NOTICE file distributed with
// this work for additional information regarding copyright ownership.  The
// ASF licenses this file to you under the Apache License, Version 2.0 (the
// "License"); you may not use this file except in compliance with the
// License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
// License for the specific language governing permissions and limitations
// under the License.
//
//***************************************************************************

//***************************************************************************
// Included Files
//***************************************************************************

#ifdef CONFIG_CXX_EXCEPTION
#include <stdexcept>
#endif

//***************************************************************************
// Private Classes
//***************************************************************************

//***************************************************************************
// Private Data
//***************************************************************************

//***************************************************************************
// Public Functions
//***************************************************************************

//***************************************************************************
// Name: exception_main
//***************************************************************************/

extern "C" int main(int argc, char *argv[])
{
#ifdef CONFIG_CXX_EXCEPTION
  try
    {
      throw std::runtime_error("runtime error");
    }
  catch (std::runtime_error &e)
    {
      if (e.what())
        {
          return 0;
        }
    }
#endif

  return 1;
}
