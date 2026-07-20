import subprocess

from hatchling.builders.hooks.plugin.interface import BuildHookInterface


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):
        # This executes right before Hatch packages your code
        print("Running pre-build bash script...")
        subprocess.run(["bash", "scripts/update_cff.sh"], check=True)
