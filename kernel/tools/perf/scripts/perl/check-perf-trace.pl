use lib "$ENV{'PERF_EXEC_PATH'}/scripts/perl/Perf-Trace-Util/lib";
use lib "./Perf-Trace-Util/lib";
use Perf::Trace::Core;
use Perf::Trace::Context;
use Perf::Trace::Util;

sub trace_begin
{
    print "trace_begin\n";
}

sub trace_end
{
    print "trace_end\n";

    print_unhandled();
}

sub irq::softirq_entry
{
	my ($event_name, $context, $common_cpu, $common_secs, $common_nsecs,
	    $common_pid, $common_comm,
	    $vec) = @_;

	print_header($event_name, $common_cpu, $common_secs, $common_nsecs,
		     $common_pid, $common_comm);

	print_uncommon($context);

	printf("vec=%s\n",
	       symbol_str("irq::softirq_entry", "vec", $vec));
}

sub kmem::kmalloc
{
	my ($event_name, $context, $common_cpu, $common_secs, $common_nsecs,
	    $common_pid, $common_comm,
	    $call_site, $ptr, $bytes_req, $bytes_alloc,
	    $gfp_flags) = @_;

	print_header($event_name, $common_cpu, $common_secs, $common_nsecs,
		     $common_pid, $common_comm);

	print_uncommon($context);

	printf("call_site=%p, ptr=%p, bytes_req=%u, bytes_alloc=%u, ".
	       "gfp_flags=%s\n",
	       $call_site, $ptr, $bytes_req, $bytes_alloc,

	       flag_str("kmem::kmalloc", "gfp_flags", $gfp_flags));
}

# print trace fields not included in handler args
sub print_uncommon
{
    my ($context) = @_;

    printf("common_preempt_count=%d, common_flags=%s, common_lock_depth=%d, ",
	   common_pc($context), trace_flag_str(common_flags($context)),
	   common_lock_depth($context));

}

my %unhandled;

sub print_unhandled
{
    if ((scalar keys %unhandled) == 0) {
	return;
    }

    print "\nunhandled events:\n\n";

    printf("%-40s  %10s\n", "event", "count");
    printf("%-40s  %10s\n", "----------------------------------------",
	   "-----------");

    foreach my $event_name (keys %unhandled) {
	printf("%-40s  %10d\n", $event_name, $unhandled{$event_name});
    }
}

sub trace_unhandled
{
    my ($event_name, $context, $common_cpu, $common_secs, $common_nsecs,
	$common_pid, $common_comm) = @_;

    $unhandled{$event_name}++;
}

sub print_header
{
	my ($event_name, $cpu, $secs, $nsecs, $pid, $comm) = @_;

	printf("%-20s %5u %05u.%09u %8u %-20s ",
	       $event_name, $cpu, $secs, $nsecs, $pid, $comm);
}