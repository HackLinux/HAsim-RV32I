
for($i = 0; $i < 10; $i = $i + 1)
{
  # use only 256-bit matrices for now
  `./generateMatrices 256 $i`;
  `./algo 256 > program.hex`;
  `./a.out > out$i.txt`;
  open(INFH,"out$i.txt" ) || die("\nCan't open out$i.txt for reading: $!\n");
  $passed = 0;
  while($line = <INFH>) 
  { 
    chomp($line);
    if($line == "PASSED")
    {
       print "Test $i passed.";
       $passed = 1;
       last;
    }
  }
  close(INFH); 

  if($passed != 1)
  {
     print "Test $i failed.";
  }  
}
