import setuptools
import platform, os, ctypes

from setuptools.dist import Distribution
from setuptools import setup, find_packages, Extension


try:
    from wheel.bdist_wheel import bdist_wheel as _bdist_wheel
    import platform, os, ctypes

    class bdist_wheel(_bdist_wheel):

        def finalize_options(self):
            _bdist_wheel.finalize_options(self)
            if platform.system() == "Darwin":
                self.root_is_pure = False

        def get_tag(self):
            python, abi, plat = _bdist_wheel.get_tag(self)
            if platform.system() == "Darwin":
                python, abi = 'py3', 'none'
                name = plat[:plat.find("_")]
                for i in range(3):
                    plat = plat[plat.find("_") + 1:]   # skip name and version of OS
                arch = plat
                version = os.getenv('MACOSX_DEPLOYMENT_TARGET').replace('.', '_')
                plat = name + "_" + version + "_" + arch
            elif platform.system() == "Windows":
                if ctypes.sizeof(ctypes.c_voidp) * 8 > 32:
                    plat = "win_" + platform.machine().lower()
                else:
                    plat = "win32"
            return python, abi, plat

except ImportError:
    bdist_wheel = None


setup_kwargs = dict(
    name='map_metrics',
    version='0.0.6',
    packages=find_packages(),
    cmdclass={'bdist_wheel': bdist_wheel}
)

setup(**setup_kwargs)
