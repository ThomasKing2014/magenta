// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <dev/iommu/intel.h>
#include "iommu_impl.h"

mxtl::RefPtr<Iommu> IntelIommu::Create(uint64_t id, paddr_t register_base) {
    return intel_iommu::IommuImpl::Create(id, register_base);
}
