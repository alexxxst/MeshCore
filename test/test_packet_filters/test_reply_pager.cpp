#include <gtest/gtest.h>
#include "ListReplyPager.h"

TEST(ListReplyPager, EmptyListPrintsNone) {
  char reply[64];
  ListReplyPager pager(reply, sizeof(reply), 0);

  pager.finish();

  EXPECT_STREQ("-none-", reply);
}

TEST(ListReplyPager, EntriesThatFitAreListedWithoutSuffix) {
  char reply[64];
  ListReplyPager pager(reply, sizeof(reply), 0);

  pager.add("aa", 2);
  pager.add("bb", 2);
  pager.finish();

  EXPECT_STREQ("aa\nbb", reply);
}

TEST(ListReplyPager, OverflowNamesTheNextPage) {
  char reply[32];
  ListReplyPager pager(reply, sizeof(reply), 0);

  for (int i = 0; i < 5; i++) pager.add("aaaa", 4);
  pager.finish();

  EXPECT_STREQ("aaaa\n+4 more (list 1)", reply);
}

TEST(ListReplyPager, OffsetSkipsEntries) {
  char reply[64];
  ListReplyPager pager(reply, sizeof(reply), 2);

  pager.add("a", 1);
  pager.add("b", 1);
  pager.add("c", 1);
  pager.add("d", 1);
  pager.add("e", 1);
  pager.finish();

  EXPECT_STREQ("c\nd\ne", reply);
}

TEST(ListReplyPager, OffsetPastTheEndPrintsNone) {
  char reply[64];
  ListReplyPager pager(reply, sizeof(reply), 5);

  pager.add("a", 1);
  pager.add("b", 1);
  pager.finish();

  EXPECT_STREQ("-none-", reply);
}

// Once an entry does not fit, the following ones must wait for the next page,
// even if they are short enough to squeeze in: otherwise the offsets drift.
TEST(ListReplyPager, EntryThatDoesNotFitDoesNotLetLaterOnesJumpThePage) {
  char reply[32];
  ListReplyPager pager(reply, sizeof(reply), 0);

  pager.add("aaaaa", 5);
  pager.add("bc", 2);
  pager.finish();

  EXPECT_STREQ("aaaaa\n+1 more (list 1)", reply);
}
