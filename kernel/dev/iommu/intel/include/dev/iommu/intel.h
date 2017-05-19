// Copyright 2017 The Fuchsia Authors
//
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#pragma once

#include <dev/iommu.h>
#include <mxtl/ref_ptr.h>

class IntelIommu {
public:
    static mxtl::RefPtr<Iommu> Create(uint64_t id, paddr_t register_base);
};
