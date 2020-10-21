import numpy as np
import os
import csv
import glob

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Create the csv from two dirs")
    parser.add_argument('a_dir', type=str, default='', help='list file with rgb/depth')
    parser.add_argument('b_dir', type=str, default='', help='list file with rgb/depth')
    parser.add_argument('out_file', type=str, default='', help='output list')
    parser.add_argument('--root', type=str, default='', help='root of dir, optional')
    parser.add_argument('--sort', action='store_true', default=False, help='root of dir, optional')

    args = parser.parse_args()

    a_dir = args.a_dir
    b_dir = args.b_dir
    root = args.root
    out_file = args.out_file
    out_root = '/'.join(out_file.split('/')[:-1])
    if os.path.isdir(out_root) is False:
        os.makedirs(out_root)
    out_txt = os.path.join(out_root, ('.'.join(out_file.split('/')[-1].split('.')[:-1]) + '.txt'))
    print('save csv file {} and txt file {}. sort({})'.format(out_file, out_txt, args.sort))

    a_names = glob.glob(os.path.join(a_dir,'*.png'))
    b_names = glob.glob(os.path.join(b_dir,'*.png'))
    if args.sort:
        a_names.sort()
        b_names.sort()

    if len(a_names) != len(b_names):
        print("len of a_names:{}!=b_names:{}\n".format(len(a_names), len(b_names)))
        return

    with open(out_file, 'w') as csvfile:
        with open(out_txt, 'w') as f:
            # spamwriter = csv.writer(csvfile, delimiter=',')
            for i in range(len(a_names)):
                a_name = '/'.join(a_names[i].split('/')[-2:])
                b_name = '/'.join(b_names[i].split('/')[-2:])
                if root != "":
                    a_name = os.path.join(root, a_name)
                    b_name = os.path.join(root, b_name)
                csvfile.write("{},{}\n".format(a_name, b_name))
                f.write("{} {}\n".format(a_name, b_name))


    return

if __name__ == "__main__":
    main()
