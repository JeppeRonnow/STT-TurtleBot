import yaml

class Config:
    # Import YAML variables on objct init
    def __init__(self) -> None:
        with open('config.yaml', 'r') as file:
            self.config = yaml.safe_load(file)

        # Go through all variables in self.config and set them as attributes
        for section, params in self.config.items():
            if isinstance(params, dict):
                for var, value in params.items():
                    setattr(self, var, value)
                    if self.DEBUG: print(f"Set attribute: {var} = {value}")
            else:
                setattr(self, section, params)
                if self.DEBUG: print(f"Set attribute: {section} = {params}")
        if self.DEBUG: print(f"[Config class initialized]")
