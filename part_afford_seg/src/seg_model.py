#!/usr/bin/env python3
import cv2
from cv2 import COLOR_BGR2RGB
from modeling.deeplab import *
from dataloaders import custom_transforms as tr
from PIL import Image
from torchvision import transforms
from dataloaders.utils import  *
from torchvision.utils import make_grid, save_image

use_cuda = False

class build_seg_model:
    def __init__(self, model="mobilenet", class_num=6, ckpt= None):
        self.seg_m = DeepLab(num_classes=class_num,
                    backbone=model,
                    output_stride=16,
                    sync_bn=None,
                    freeze_bn=False)
        ckpt = torch.load(ckpt, map_location='cpu')
        self.seg_m.load_state_dict(ckpt['state_dict'])

        self.composed_transforms = transforms.Compose([
            tr.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
            tr.ToTensor()])

        if use_cuda == True:
            self.seg_m.cuda()
        self.seg_m.eval()
    
    def run(self, roi_img):
        roi = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)
        roi = cv2.resize(roi,(224,224))
        roi = Image.fromarray(roi)

        image = roi.convert("RGB")
        target = roi.convert("L")

        sample = {'image': image, 'label': target}
        tensor_in = self.composed_transforms(sample)['image'].unsqueeze(0)

        # image = image.cuda()
        with torch.no_grad():
            output = self.seg_m(tensor_in.cuda())

        grid_image = make_grid(decode_seg_map_sequence(torch.max(output[:3], 1)[1].detach().cpu().numpy()), 
                            3, normalize=False, range=(0, 255))
        
        grid_image = grid_image.permute(1,2,0)
        grid_image = np.array(grid_image)
        # # print(grid_image.shape)
        grid_image = grid_image[...,::-1]


        return grid_image

if __name__ == "__main__":
    seg_m = build_seg_model(model="mobilenet", class_num=6, ckpt="./111_project/components.pth")
    img = cv2.imread("demo/motor_2_1.jpg")
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    res = seg_m.run(img)
    cv2.imshow("1", res)
    cv2.waitKey(0)

