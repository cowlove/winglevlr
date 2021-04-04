#!/usr/bin/perl

# time ./sweep.pl NUM=107 sweep.txt 

if ( $ARGV[0] =~ /NUM=(\d+)/i ) {
	$num = $1;  # Allow NUM= to be overriden on the command line
	shift;
}

while(<>) {
	if (/^\s*[#]/ || /^s*$/) { 
		#comment  
    } elsif (/ARGS=(.+)/i) { 
		$args = $1;
    } elsif (/FIELD=(\d+)/i) { 
		$field = $1;
	} elsif (/NUM=(\d+)/i) {
		if (! defined $num) { 
			$num = $1;
		}
	} else { 
		($param, $mid, $span) = split(' ', $_);
		$span{$param} = $span;
		$mid{$param} = $mid;
		$bestV{$param} = $mid;
		push(@params, $param);
	}
}

$round = 1;	
while($round < 100) { 
	$it = 1;
	$lastBest = -1;
	while(($#params >= 2 && $it < 10) || ($#params < 2 && $it < 2)) { 
		foreach $t ( @params ) {
			delete $best{$t};
			$span = $span{$t} / $round;
			$inc = $span / 5;
			$lo = $mid{$t} - $span;
			$hi = $mid{$t} + $span;
			print "Sweep '$t' $lo to $hi by $inc, iteration $it:\n";
			for ($v = $lo; $v <= $hi; $v = $v + $inc) {  
				$valstr = ",";
				foreach $p ( @params ) {
					if (exists $bestV{$p} && ($t ne $p)) { 
						$valstr = $valstr . "$p=" . sprintf("%.7f", $bestV{$p}) . ",";
					} 
					if ($t eq $p) {
						$valstr = $valstr . "$p=". sprintf("%.7f", $v).",";
					} 
				}
				$cmd = "./winglevlr_ubuntu  $args --replay ./logs/AHRSD$num.DAT --debug \"$valstr\"";
				print $cmd . " :\t";
				if (`$cmd` =~ /#\s([0-9.+-]+)\s([0-9.+-]+)\s([0-9.+-]+)/ ) {
					if ($field == 1) {
						$result = $1;
					} elsif ($field == 2) { 
						$result = $2;
					} elsif ($field == 3) { 
						$result = $3;
					}
				}
				$totalRuns += 1;
				print  $result;
				if (!exists $best{$t} || $result <= $best{$t}) {
					print " *"; 
					$best{$t} = $result;
					$bestV{$t} = $v;
					$bestCmd = $cmd;
					$bestScore = $result;
				}
				print "\n";
			}
		}
		if ($bestScore == $lastBest) {
			print "No improvement, quitting.\n";
			last;
		}
		$lastBest = $bestScore;
		$it += 1;
	}
	print "Result: $bestScore\tIterations: Round: $round\t$it\tTotal simulations: $totalRuns\n$bestCmd\n\n";
	$round *= 5;
	foreach $t (keys %bestV) {
		$mid{$t} = $bestV{$t}
	}
}

