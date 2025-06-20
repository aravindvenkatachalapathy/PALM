#!/usr/bin/env python3
"""
PALM Input File Generator
Reads PALM p3d input files and generates multiple simulation files with modified parameters.
"""

import re
import os
import copy
from pathlib import Path
from typing import Dict, List, Any, Union


class PALMInputParser:
    """Parser for PALM input files in Fortran namelist format."""
    
    def __init__(self):
        self.namelists = {}
        self.comments = {}
        self.original_content = ""
    
    def read_input_file(self, filename: str) -> Dict[str, Dict[str, Any]]:
        """
        Read and parse a PALM input file.
        
        Args:
            filename: Path to the input file
            
        Returns:
            Dictionary containing all namelists and their parameters
        """
        with open(filename, 'r') as f:
            self.original_content = f.read()
        
        # Split content into namelists
        namelist_pattern = r'&(\w+)(.*?)/'
        matches = re.findall(namelist_pattern, self.original_content, re.DOTALL)
        
        for namelist_name, content in matches:
            self.namelists[namelist_name] = self._parse_namelist_content(content)
        
        return self.namelists
    
    def _parse_namelist_content(self, content: str) -> Dict[str, Any]:
        """Parse the content of a single namelist."""
        parameters = {}
        
        # Remove comments and clean up
        lines = content.split('\n')
        for line in lines:
            # Remove inline comments
            if '!' in line:
                line = line[:line.index('!')]
            
            line = line.strip()
            if not line or line.startswith('!'):
                continue
            
            # Handle parameter assignments
            if '=' in line:
                # Handle multi-line arrays
                if line.endswith(','):
                    # This is part of a multi-line parameter
                    param_name = line.split('=')[0].strip()
                    values = []
                    
                    # Collect all values for this parameter
                    value_part = line.split('=')[1].strip()
                    if value_part.endswith(','):
                        value_part = value_part[:-1]
                    
                    values.extend(self._parse_values(value_part))
                    parameters[param_name] = values if len(values) > 1 else values[0] if values else None
                else:
                    # Single line parameter
                    parts = line.split('=', 1)
                    if len(parts) == 2:
                        param_name = parts[0].strip()
                        value_str = parts[1].strip().rstrip(',')
                        values = self._parse_values(value_str)
                        parameters[param_name] = values if len(values) > 1 else values[0] if values else None
        
        return parameters
    
    def _parse_values(self, value_str: str) -> List[Any]:
        """Parse parameter values, handling different data types."""
        if not value_str:
            return []
        
        # Handle string values
        if value_str.startswith("'") and value_str.endswith("'"):
            return [value_str.strip("'")]
        
        # Handle boolean values
        if value_str.lower() in ['.true.', '.false.']:
            return [value_str.lower() == '.true.']
        
        # Handle numeric arrays (comma-separated)
        if ',' in value_str:
            values = []
            for val in value_str.split(','):
                val = val.strip()
                if val:
                    values.append(self._convert_value(val))
            return values
        
        # Single value
        return [self._convert_value(value_str)]
    
    def _convert_value(self, value_str: str) -> Union[int, float, str, bool]:
        """Convert string value to appropriate Python type."""
        value_str = value_str.strip()
        
        if not value_str:
            return None
        
        # Boolean
        if value_str.lower() in ['.true.', '.false.']:
            return value_str.lower() == '.true.'
        
        # String
        if value_str.startswith("'") and value_str.endswith("'"):
            return value_str.strip("'")
        
        # Try numeric conversion
        try:
            if '.' in value_str:
                return float(value_str)
            else:
                return int(value_str)
        except ValueError:
            return value_str


class PALMInputGenerator:
    """Generator for creating multiple PALM input files with parameter variations."""
    
    def __init__(self, base_file: str):
        self.parser = PALMInputParser()
        self.base_parameters = self.parser.read_input_file(base_file)
        self.base_filename = Path(base_file).stem
    
    def get_parameter(self, namelist: str, parameter: str) -> Any:
        """Get a specific parameter value."""
        return self.base_parameters.get(namelist, {}).get(parameter)
    
    def set_parameter(self, namelist: str, parameter: str, value: Any) -> None:
        """Set a specific parameter value."""
        if namelist not in self.base_parameters:
            self.base_parameters[namelist] = {}
        self.base_parameters[namelist][parameter] = value
    
    def generate_parameter_study(self, parameter_variations: Dict[str, Dict[str, List[Any]]], 
                               output_dir: str = "./simulations") -> List[str]:
        """
        Generate multiple input files for parameter study.
        
        Args:
            parameter_variations: Dictionary with namelist -> parameter -> list of values
            output_dir: Directory to save generated files
            
        Returns:
            List of generated filenames
        """
        os.makedirs(output_dir, exist_ok=True)
        generated_files = []
        
        # Generate all combinations
        combinations = self._generate_combinations(parameter_variations)
        
        for i, combo in enumerate(combinations):
            # Create modified parameters
            modified_params = copy.deepcopy(self.base_parameters)
            filename_suffix = []
            
            for namelist, param_dict in combo.items():
                for param, value in param_dict.items():
                    modified_params[namelist][param] = value
                    filename_suffix.append(f"{param}_{value}")
            
            # Generate filename
            suffix = "_".join(filename_suffix)
            output_filename = f"{self.base_filename}_{i+1:03d}_{suffix}_p3d"
            output_path = os.path.join(output_dir, output_filename)
            
            # Write file
            self._write_input_file(modified_params, output_path)
            generated_files.append(output_path)
            
            print(f"Generated: {output_filename}")
        
        return generated_files
    
    def _generate_combinations(self, variations: Dict[str, Dict[str, List[Any]]]) -> List[Dict]:
        """Generate all combinations of parameter variations."""
        from itertools import product
        
        # Flatten variations into a list of (namelist, param, values) tuples
        variation_items = []
        for namelist, params in variations.items():
            for param, values in params.items():
                variation_items.append((namelist, param, values))
        
        # Generate all combinations
        combinations = []
        if variation_items:
            value_lists = [item[2] for item in variation_items]
            for combo in product(*value_lists):
                combination_dict = {}
                for i, (namelist, param, _) in enumerate(variation_items):
                    if namelist not in combination_dict:
                        combination_dict[namelist] = {}
                    combination_dict[namelist][param] = combo[i]
                combinations.append(combination_dict)
        
        return combinations
    
    def _write_input_file(self, parameters: Dict[str, Dict[str, Any]], filename: str) -> None:
        """Write parameters to a PALM input file."""
        with open(filename, 'w') as f:
            # Write header comment
            f.write("!-------------------------------------------------------------------------------\n")
            f.write("!-- PALM INPUT FILE GENERATED BY PYTHON SCRIPT\n")
            f.write("!-------------------------------------------------------------------------------\n\n")
            
            # Write each namelist
            for namelist_name, params in parameters.items():
                f.write(f"!-------------------------------------------------------------------------------\n")
                f.write(f"!-- {namelist_name.upper()} \n")
                f.write(f"!-------------------------------------------------------------------------------\n")
                f.write(f"&{namelist_name}\n")
                
                # Write parameters
                for param_name, value in params.items():
                    if value is not None:
                        formatted_value = self._format_value(value)
                        f.write(f"    {param_name:<30} = {formatted_value},\n")
                
                f.write("\n/\n\n")
    
    def _format_value(self, value: Any) -> str:
        """Format a value for writing to the input file."""
        if isinstance(value, bool):
            return '.TRUE.' if value else '.FALSE.'
        elif isinstance(value, str):
            return f"'{value}'"
        elif isinstance(value, list):
            formatted_items = []
            for item in value:
                if isinstance(item, str):
                    formatted_items.append(f"'{item}'")
                elif isinstance(item, bool):
                    formatted_items.append('.TRUE.' if item else '.FALSE.')
                else:
                    formatted_items.append(str(item))
            return ', '.join(formatted_items)
        else:
            return str(value)


# Example usage
if __name__ == "__main__":
    # Initialize the generator with your base file
    generator = PALMInputGenerator("C:/Users/varav/Downloads/E-4.1_Land_surface_model_steering_files_complete/E-4_Land_surface_model_steering_files_complete/e4_LSM1/INPUT/e4_LSM1_p3d")
    
    # Print current parameters (optional)
    print("Base parameters loaded:")
    for namelist, params in generator.base_parameters.items():
        print(f"\n{namelist}:")
        for param, value in params.items():
            print(f"  {param}: {value}")
    
    # Define parameter variations for your study
    parameter_variations = {
        'initialization_parameters': {
            'nx': [39, 79, 119],  # Different grid resolutions
            'ny': [39, 79, 119],
            'pt_surface': [293.0, 295.0, 298.0],  # Different surface temperatures
        },
        'runtime_parameters': {
            'end_time': [3600.0, 7200.0, 10800.0],  # Different simulation times
        },
        'radiation_parameters': {
            'net_radiation': [600.0, 800.0, 1000.0],  # Different radiation values
        }
    }
    
    # Generate the parameter study files
    print("\nGenerating parameter study files...")
    generated_files = generator.generate_parameter_study(
        parameter_variations, 
        output_dir="./palm_simulations"
    )
    
    print(f"\nGenerated {len(generated_files)} simulation files:")
    for file in generated_files:
        print(f"  {file}")
    
    # Example: Generate a single modified file
    print("\nGenerating single modified file...")
    generator.set_parameter('initialization_parameters', 'nx', 79)
    generator.set_parameter('initialization_parameters', 'ny', 79)
    generator.set_parameter('runtime_parameters', 'end_time', 3600.0)
    
    single_file_params = copy.deepcopy(generator.base_parameters)
    generator._write_input_file(single_file_params, "test_modified_p3d")
    print("Generated: test_modified_p3d")