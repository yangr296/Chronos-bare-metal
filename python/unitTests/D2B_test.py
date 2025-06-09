import unittest
import math
import sys
import os

# Add the src directory to the path so we can import D2B
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
import D2B

class TestD2B(unittest.TestCase):
    
    def setUp(self):
        """Set up test constants"""
        self.V_REF = 3.3
        self.R_SENSE = 100
        self.MAX_CURRENT = math.floor(3.3 * 10 / self.R_SENSE * 1000000)  # in μA
        
    def test_zero_current(self):
        """Test that 0 μA gives 0x8000 (0V output)"""
        result = D2B.decimal_to_binary(0)
        self.assertEqual(result, 0x8000, 
                        f"0 μA should give 0x8000, got 0x{result:04X}")
    
    def test_maximum_positive_current(self):
        """Test maximum positive current (should approach 0xFFFF)"""
        # Maximum current that gives +V_REF * (32767/32768)
        max_current = self.MAX_CURRENT
        result = D2B.decimal_to_binary(max_current)
        # Should be close to 0xFFFF (65535) but not exceed it
        self.assertGreaterEqual(result, 0xFFFE, 
                              f"Max current {max_current} μA should give ~0xFFFF, got 0x{result:04X}")
        self.assertLessEqual(result, 0xFFFF)
    
    def test_maximum_negative_current(self):
        """Test maximum negative current (should approach 0x0000)"""
        # Maximum negative current
        max_neg_current = -self.MAX_CURRENT
        result = D2B.decimal_to_binary(max_neg_current)
        # Should be close to 0x0000
        self.assertLessEqual(result, 0x0001, 
                           f"Max negative current {max_neg_current} μA should give ~0x0000, got 0x{result:04X}")
        self.assertGreaterEqual(result, 0x0000)
    
    def test_positive_one_lsb(self):
        """Test +1 LSB value (+V_REF * 1/32768)"""
        # Calculate current for 1 LSB above zero
        # V_DAC = V_REF * (1/32768)
        # I = 10 * V_DAC / R_SENSE (in A)
        # I_uA = I * 1000000
        v_dac_1lsb = self.V_REF * (1.0/32768.0)
        current_1lsb = (10 * v_dac_1lsb / self.R_SENSE) * 1000000  # in μA
        
        result = D2B.decimal_to_binary(current_1lsb)
        self.assertEqual(result, 0x8001, 
                        f"Current for +1 LSB ({current_1lsb:.6f} μA) should give 0x8001, got 0x{result:04X}")
    
    def test_negative_one_lsb(self):
        """Test -1 LSB value (-V_REF * 1/32768)"""
        # Calculate current for 1 LSB below zero
        v_dac_neg1lsb = -self.V_REF * (1.0/32768.0)
        current_neg1lsb = (10 * v_dac_neg1lsb / self.R_SENSE) * 1000000  # in μA
        
        result = D2B.decimal_to_binary(current_neg1lsb)
        self.assertEqual(result, 0x7FFF, 
                        f"Current for -1 LSB ({current_neg1lsb:.6f} μA) should give 0x7FFF, got 0x{result:04X}")
    
    def test_half_scale_positive(self):
        """Test half-scale positive value"""
        # Half of maximum positive current
        half_current = self.MAX_CURRENT / 2
        result = D2B.decimal_to_binary(half_current)
        # Should be around 0xC000 (0x8000 + 0x4000)
        expected = 0xC000
        tolerance = 0x0001  # Allow 1 LSB tolerance
        self.assertAlmostEqual(result, expected, delta=tolerance,
                             msg=f"Half scale positive ({half_current} μA) should give ~0x{expected:04X}, got 0x{result:04X}")
    
    def test_half_scale_negative(self):
        """Test half-scale negative value"""
        # Half of maximum negative current
        half_neg_current = -self.MAX_CURRENT / 2
        result = D2B.decimal_to_binary(half_neg_current)
        # Should be around 0x4000 (0x8000 - 0x4000)
        expected = 0x4000
        tolerance = 0x0001  # Allow 1 LSB tolerance
        self.assertAlmostEqual(result, expected, delta=tolerance,
                             msg=f"Half scale negative ({half_neg_current} μA) should give ~0x{expected:04X}, got 0x{result:04X}")
    
    def test_common_values(self):
        """Test common stimulation current values"""
        test_cases = [
            (100, "100 μA"),      # 100 μA
            (500, "500 μA"),      # 500 μA
            (1000, "1 mA"),       # 1 mA
            (2000, "2 mA"),       # 2 mA
            (-100, "-100 μA"),    # -100 μA
            (-500, "-500 μA"),    # -500 μA
            (-1000, "-1 mA"),     # -1 mA
            (-2000, "-2 mA"),     # -2 mA
        ]
        
        for current, description in test_cases:
            result = D2B.decimal_to_binary(current)
            # Verify result is in valid range
            self.assertGreaterEqual(result, 0x0000, 
                                  f"{description} gave invalid result: 0x{result:04X}")
            self.assertLessEqual(result, 0xFFFF, 
                               f"{description} gave invalid result: 0x{result:04X}")
            
            # Verify direction (positive current > 0x8000, negative < 0x8000)
            if current > 0:
                self.assertGreater(result, 0x8000, 
                                 f"Positive current {description} should give value > 0x8000, got 0x{result:04X}")
            elif current < 0:
                self.assertLess(result, 0x8000, 
                              f"Negative current {description} should give value < 0x8000, got 0x{result:04X}")
    
    def test_clamping(self):
        """Test that values beyond ±MAX_CURRENT are clamped"""
        # Test overcurrent positive
        overcurrent = self.MAX_CURRENT * 2
        result = D2B.decimal_to_binary(overcurrent)
        self.assertEqual(result, 0xFFFF, 
                        f"Overcurrent {overcurrent} μA should be clamped to 0xFFFF, got 0x{result:04X}")
        
        # Test overcurrent negative
        overcurrent_neg = -self.MAX_CURRENT * 2
        result = D2B.decimal_to_binary(overcurrent_neg)
        self.assertEqual(result, 0x0000, 
                        f"Negative overcurrent {overcurrent_neg} μA should be clamped to 0x0000, got 0x{result:04X}")

if __name__ == '__main__':
    # Run with verbose output to see print statements from D2B
    unittest.main(verbosity=2)