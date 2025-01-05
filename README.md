# Data Packet Solver

Packet encapsulation and parsing module, provides data packet construction and analysis functionalities.

---

## Build and Run
---

## Important Note

Please ensure that the buffer size meets the requirements before using.  
- **For the C version version, modify the `BUFFER_SIZE` macro in `DataPacketSolver.h`.**
- **For the C++ version, specify the template parameter during construction.**

---
### Build Steps

1. It is recommended to use a C++17-compliant compiler and `CMake` (>= 3.0.2).  
  
2. Execute the following commands to build the project:
   ```bash
   mkdir build
   cd build
   cmake ..
   make
   ```

3. After a successful build, the executable `data_packet_solver` will be available in the `build` directory.

---

### Run Tests

Run the executable to test the module:
```bash
./data_packet_solver
```

---

## License

```text
Copyright (c) 2024, ADrownFish. 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
