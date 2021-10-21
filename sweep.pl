#!/usr/bin/perl

# time ./sweep.pl NUM=107 sweep.txt 
$exhaustive = 1;
$zoom = 2;
$goal = 0.0001;
if ( $ARGV[0] =~ /(\d+)/i ) {
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
		if (length($param) > 1) { 
			$span{$param} = $span;
			$mid{$param} = $mid;
			$bestV{$param} = $mid;
			push(@params, $param);
		}
	}
}

sub process { 
	my ($dstr, $pidx) = @_;
	my $t = $params[$pidx];
	my $span = $span{$t} / $round;
	my $inc = $span / $zoom;
	my $lo = $mid{$t} - $span;
	my $hi = $mid{$t} + $span;
	if (length($t) == 0) {
		return;
	}
	my $v;
	print "Sweep $pidx/'$t' $lo to $hi by $inc:\n";
	for ($v = $lo; $v <= $hi; $v = $v + $inc) {  
		my $valstr = ",";
		$currentParamValues{$t} = $v;
		if ($pidx < scalar(@params) -1) {
			process($astr, $pidx + 1);
		} else { 
			$a = "";
			foreach $p (@params) { 
				$a = $a . "$p=" . sprintf("%.9f,", $currentParamValues{$p});
			}
			my $cmd = "./winglevlr_ubuntu  $args --replay ./logs/AHRSD$num.DAT --debug \"$a\"";
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
			if (($bestScore == 0 || $result < $bestScore) && $result > 0) { 
				$bestScore = $result;
				$bestCmd = $cmd;
				foreach $p (@params) { 
					$bestParamValues{$p} = $currentParamValues{$p};
				}
				print " *";
			}
			print "\n";
		}
	}
}

$round = 1;	
$bestCmd = "";
$secondBest = -1;

while(1) { 
	if ($exhaustive) { 
		process("", 0);
		foreach $p (@params) {
			$mid{$p} = $bestParamValues{$p};
		} 
		print "RESULTS: $bestScore after $totalRuns runs\n\n$bestCmd\n";

		if (abs($bestScore - $lastRoundBest) < $goal) { 
			print "Met goal, quitting.\n";
			print "Result: $bestScore\tTotal simulations: $totalRuns\n$bestCmd\n\n";
			last;
		}
		$lastRoundBest = $bestScore;
		$round *= $zoom;
	
	} else {
		$it = 1;
		while(($#params >= 2 && $it < 10) || ($#params < 2 && $it < 3)) { 
			foreach $t ( @params ) {
				delete $best{$t};
				$span = $span{$t} / $round;
				$inc = $span / 5;
				$lo = $mid{$t} - $span - $inc * 2;
				$hi = $mid{$t} + $span + $inc * 2;
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
						$secondBest = $bestScore;
						$bestScore = $result;
					}
					print "\n";
					if ($inc == 0) {
						last;
					}
				}
			}
			if (abs($bestScore - $secondBest) < $goal) { 
				print "Met goal, quitting.\n";
				last;
			}
			if ($bestScore == $secondBest) {
				print "No improvement, quitting.\n";
				last;
			}
			$secondBest = $bestScore;
			$it += 1;
		}
		print "Result: $bestScore\tIterations: Round: $round\t$it\tTotal simulations: $totalRuns\n$bestCmd\n\n";
		if (abs($bestScore - $secondBest) < $goal) { 
			print "Met goal, quitting.\n";
			last;
		}
		$lastRoundBest = $bestScore;
		$round *= 5;
		foreach $t (keys %bestV) {
			$mid{$t} = $bestV{$t}
		}
	}
}

