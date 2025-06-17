d <- ggplot(diamonds, aes(carat, price))
p <-  d + geom_hex()