# autoware_utils

Common utilities for Autoware.

## Usage

To use all features, include `autoware_utils/autoware_utils.hpp`:

```cpp
#include <autoware_utils/autoware_utils.hpp>

using autoware_utils::deg2rad;
using autoware_utils::normalize_degree;
using autoware_utils::pi;
```

To select features, include necessary header files:

```cpp
#include <autoware_utils/math/constants.hpp>
#include <autoware_utils/math/normalization.hpp>

using autoware_utils::normalize_degree;
using autoware_utils::pi;
```
