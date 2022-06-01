"""
Copyright (c) 2022 Teslabs Engineering S.L.
SPDX-License-Identifier: Apache 2.0
"""

from gd32headers import main


def test_main(data, hal_path, tmp_path):
    """Test that common headers are generated correctly."""

    main(hal_path, tmp_path)

    # check adc file
    ref_adc_file = data / "gd32_adc.h"
    gen_adc_file = tmp_path / "gd32_adc.h"

    assert gen_adc_file.exists()

    with open(ref_adc_file) as ref, open(gen_adc_file) as gen:
        assert ref.read() == gen.read()

    # check libopt file is not created (not an API)
    gen_libopt_file = tmp_path / "gd32_libopt.h"

    assert not gen_libopt_file.exists()
