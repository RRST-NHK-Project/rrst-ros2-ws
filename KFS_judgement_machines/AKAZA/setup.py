from setuptools import setup

package_name = "kfs_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    package_dir={"": "."},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/kfs_pkg"]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/resource",
            [
                "resource/KFS_image_list.png",
                "resource/KFS_image_list_all.png",
                "resource/KFS_judgement_machine ver1.1.pth",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@test.com",
    description="KFS package",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # matcher_node.py 内の main 関数を呼び出す
            "matcher_node = kfs_pkg.matcher_node:main",
            "udp_node = kfs_pkg.node:main",
        ],
    },
)
