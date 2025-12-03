import unittest
import numpy as np
import FP_funcs as Func
import config
from FP_funcs import Terrain, TerrainType

class TestFPFuncs(unittest.TestCase):
    def test_heuristic(self):
        a = (0, 0)
        b = (3, 4)
        self.assertEqual(Func.heuristic(a, b), 5.0)

    def test_get_neighbors(self):
        # Corner case (0,0)
        neighbors = Func.get_neighbors((0, 0), 10)
        self.assertEqual(set(neighbors), {(0, 1), (1, 0)})
        
        # Middle case (5,5)
        neighbors = Func.get_neighbors((5, 5), 10)
        self.assertEqual(set(neighbors), {(4, 5), (6, 5), (5, 4), (5, 6)})
        
        # Edge case (9,9)
        neighbors = Func.get_neighbors((9, 9), 10)
        self.assertEqual(set(neighbors), {(8, 9), (9, 8)})

    def test_coordinate_conversion(self):
        resolution = 100
        r = config.WORLD_SIZE / resolution # 0.1
        
        # Center of map (50, 50) should be (0, 0) in world
        world_pos = Func.convert_map_to_world(50, 50, r, resolution)
        self.assertAlmostEqual(world_pos[0], 0.0)
        self.assertAlmostEqual(world_pos[1], 0.0)
        
        map_pos = Func.convert_world_to_map(0.0, 0.0, r, resolution)
        self.assertEqual(map_pos, [50, 50])
        
        # Test offset
        # i=60 -> (60-50)*0.1 = 1.0
        world_pos = Func.convert_map_to_world(60, 50, r, resolution)
        self.assertAlmostEqual(world_pos[0], 1.0)
        
        map_pos = Func.convert_world_to_map(1.0, 0.0, r, resolution)
        self.assertEqual(map_pos, [60, 50])

    def test_mask_color(self):
        # Create dummy image (2x2)
        img = np.zeros((2, 2, 3), dtype=np.uint8)
        img[0, 0] = [255, 0, 0] # Red
        img[0, 1] = [0, 255, 0] # Green
        img[1, 0] = [0, 0, 255] # Blue
        img[1, 1] = [255, 255, 255] # White
        
        mask_red = Func.mask_color(img, "red")
        self.assertTrue(mask_red[0, 0])
        self.assertFalse(mask_red[0, 1])
        self.assertFalse(mask_red[1, 0])
        self.assertFalse(mask_red[1, 1])
        
        mask_green = Func.mask_color(img, "green")
        self.assertTrue(mask_green[0, 1])
        
        mask_blue = Func.mask_color(img, "blue")
        self.assertTrue(mask_blue[1, 0])

    def test_centroid(self):
        mask = np.zeros((10, 10), dtype=bool)
        mask[5, 5] = True
        mask[5, 6] = True
        mask[6, 5] = True
        mask[6, 6] = True
        
        centroid = Func.centroid_from_mask(mask)
        self.assertAlmostEqual(centroid[0], 5.5) # x (col)
        self.assertAlmostEqual(centroid[1], 5.5) # y (row)

    def test_terrain(self):
        t = Terrain(width=1, coordinate=[0, 0], terrain_type=TerrainType.GRASS, resolution=100)
        self.assertEqual(t.get_terrain_cost(), 2)
        self.assertEqual(t.get_terrain_num(), 1)
        self.assertFalse(t.is_floor())
        self.assertFalse(t.is_obstacle())
        
        # Check coordinate storage
        self.assertEqual(t.i, 50)
        self.assertEqual(t.j, 50)
        
        wc = t.get_world_coords()
        self.assertAlmostEqual(wc[0], 0.0)
        self.assertAlmostEqual(wc[1], 0.0)

if __name__ == '__main__':
    unittest.main()
