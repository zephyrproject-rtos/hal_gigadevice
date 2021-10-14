
from gd32pinctrl import main


def test_main(data, tmp_path):
    """Test that pinctrl headers are generated correctly."""

    main(data, tmp_path)

    FILES = (
        "gd32f888x(0-1)xx-pinctrl.h",
        "gd32f888x(2-3)xx-pinctrl.h",
        "gd32f888y(0-1)xx-pinctrl.h",
        "gd32f999x(0-1)xx-pinctrl.h",
        "gd32f999x(2-3)xx-pinctrl.h",
        "gd32f999y(0-1)xx-pinctrl.h",
    )

    for file in FILES:
        ref_file = data / file
        gen_file = tmp_path / file

        assert gen_file.exists()

        with open(ref_file) as ref, open(gen_file) as gen:
            assert ref.read() == gen.read()
