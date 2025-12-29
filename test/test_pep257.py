# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause


from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.'])
    assert rc == 0, 'Found code style errors'
