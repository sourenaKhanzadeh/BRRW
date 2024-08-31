
    #!/bin/bash
    for j in {1..30}
    do
      for i in {0..29}
      do

          python3 Xperiment102.py --domain 0 --problem $j --task $i
      done
    done